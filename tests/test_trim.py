"""二次侧配平：锁死规则、保远端裁决、串行执行与部分失败。"""

from __future__ import annotations

import threading

import pytest

from heatnet.console.api import Api
from heatnet.secondary.actuator import (
    SimulatedTrimDriver,
    TrimActuatorBus,
    execute_trim,
)
from heatnet.secondary.trim import (
    METHOD_EQUAL_DELTA_T,
    PRIORITY_FAR_FIRST,
    ROW_CLAMPED_LOW,
    ROW_READBACK_MISSING,
    ROW_TELEMETRY_MISSING,
    STATUS_OK,
    STATUS_PARTIAL,
    TRIM_MAX_TRAVEL_PCT,
    build_trim_plan,
)
from heatnet.secondary.trim_service import TrimService

DEMO_READINGS = [
    {"station": "st01", "supply_c": 65.0, "return_c": 50.0, "flow_t_h": 300.0, "valve_pct": 80.0},
    {"station": "st02", "supply_c": 65.0, "return_c": 48.0, "flow_t_h": 260.0, "valve_pct": 70.0},
    {"station": "st03", "supply_c": 65.0, "return_c": None, "flow_t_h": 220.0, "valve_pct": 60.0},
    {"station": "st04", "supply_c": 65.0, "return_c": 44.0, "flow_t_h": 210.0, "valve_pct": 55.0},
    {"station": "st05", "supply_c": 65.0, "return_c": 42.0, "flow_t_h": 180.0, "valve_pct": 45.0},
    {"station": "st06", "supply_c": 65.0, "return_c": 40.0, "flow_t_h": 150.0, "valve_pct": 35.0},
]


def _driver_for(readings) -> SimulatedTrimDriver:
    driver = SimulatedTrimDriver()
    for raw in readings:
        if raw["return_c"] is not None:
            driver.seed(raw["station"], raw["valve_pct"])
    return driver


def test_method_and_priority_are_locked():
    plan = build_trim_plan(DEMO_READINGS)
    assert plan.method == METHOD_EQUAL_DELTA_T
    assert plan.method == "EQUAL_DELTA_T"
    assert plan.priority == PRIORITY_FAR_FIRST
    assert plan.priority == "FAR_FIRST"


def test_missing_readback_station_is_excluded_not_balanced():
    plan = build_trim_plan(DEMO_READINGS)
    missing = plan.get("st03")
    assert missing.status == ROW_TELEMETRY_MISSING
    assert missing.target_flow_t_h is None
    assert missing.target_valve_pct is None
    assert plan.missing_stations == ("st03",)
    assert plan.participant_count == 5
    # 均值只由 5 个参与站算出，缺测站既不抬也不压。
    assert plan.target_delta_c == pytest.approx(21220.0 / 1100.0, abs=1e-3)
    total = sum(
        row.target_flow_t_h for row in plan.rows if row.target_flow_t_h is not None
    )
    assert total == pytest.approx(1100.0)


def test_equal_delta_target_pulls_far_and_throttles_near():
    plan = build_trim_plan(DEMO_READINGS)
    near = plan.get("st01")
    far = plan.get("st06")
    # 近端压水、关阀；远端抬水、开阀。
    assert near.target_flow_t_h < near.flow_t_h
    assert near.target_valve_pct < near.valve_pct
    assert far.target_flow_t_h > far.flow_t_h
    assert far.target_valve_pct > far.valve_pct
    # 全部参与站一轮行程内可达。
    for row in plan.rows:
        if row.valve_move_pct is not None:
            assert abs(row.valve_move_pct) <= TRIM_MAX_TRAVEL_PCT


def test_fingerprint_is_stable_and_sealed():
    first = build_trim_plan(DEMO_READINGS).as_dict()
    second = build_trim_plan(DEMO_READINGS).as_dict()
    assert first["fingerprint"] == second["fingerprint"]
    assert len(first["fingerprint"]) == 16
    tampered = [dict(DEMO_READINGS[0], flow_t_h=999.0)] + DEMO_READINGS[1:]
    assert build_trim_plan(tampered).fingerprint != first["fingerprint"]


def test_far_first_pins_far_station_when_valve_cannot_open_enough():
    # 最远端当前阀已近全开、流量却小：开不到等温差要求的水量，应先保远端
    # （钉在可行域上沿），近端吸收残余。
    readings = [
        {"station": "near", "supply_c": 60.0, "return_c": 50.0, "flow_t_h": 300.0, "valve_pct": 80.0},
        {"station": "mid", "supply_c": 60.0, "return_c": 45.0, "flow_t_h": 200.0, "valve_pct": 50.0},
        {"station": "far", "supply_c": 60.0, "return_c": 30.0, "flow_t_h": 60.0, "valve_pct": 98.0},
    ]
    plan = build_trim_plan(readings)
    far = plan.get("far")
    # 远端想要更多水但阀已近全开、开不到目标，被钉在可行域上沿（下沿截断，
    # 缺口按远端优先向前转嫁给中/近端）。
    assert far.status == ROW_CLAMPED_LOW
    assert far.target_flow_t_h == pytest.approx(62.39, abs=0.1)
    # 远端先保：中/近端吸收了远端没拿到的水量，仍在压近端的水。
    assert plan.get("near").target_flow_t_h < plan.get("near").flow_t_h
    # 中途钉死但近端兜住了全部残余水量，方案仍可行（无总流量短欠）。
    assert plan.flow_shortfall_t_h == 0.0
    assert plan.feasible is True


def test_infeasible_when_last_station_cannot_hold_budget():
    readings = [
        {"station": "near", "supply_c": 60.0, "return_c": 30.0, "flow_t_h": 400.0, "valve_pct": 10.0},
        {"station": "far", "supply_c": 60.0, "return_c": 58.0, "flow_t_h": 400.0, "valve_pct": 90.0},
    ]
    plan = build_trim_plan(readings)
    assert plan.feasible is False
    assert plan.flow_shortfall_t_h == pytest.approx(150.0)
    assert plan.get("near").status == ROW_CLAMPED_LOW


def test_execute_runs_far_first_serially():
    plan = build_trim_plan(DEMO_READINGS)
    driver = _driver_for(DEMO_READINGS)
    run = execute_trim(plan, driver)
    assert run.status == STATUS_OK
    assert run.reconciled is True
    orders = [step["order"] for step in run.steps]
    assert orders == sorted(orders, reverse=True)
    assert [step["station"] for step in run.steps][0] == "st06"
    # 每一步都有独立序号且回读到站。
    assert [step["seq"] for step in run.steps] == list(range(1, len(run.steps) + 1))


def test_readback_missing_is_partial_failure_station_held():
    plan = build_trim_plan(DEMO_READINGS)
    driver = _driver_for(DEMO_READINGS)
    driver.fail_readback("st02")
    run = execute_trim(plan, driver)
    assert run.status == STATUS_PARTIAL
    assert run.readback_missing_stations == ("st02",)
    held = next(item for item in run.results if item.station == "st02")
    assert held.status == ROW_READBACK_MISSING
    assert held.actual_valve_pct == pytest.approx(70.0)
    assert held.moved is False
    # 其余站继续并对账一致（缺测站没动、回读缺失站保持原值）。
    assert run.reconciled is True
    far = next(item for item in run.results if item.station == "st06")
    assert far.actual_valve_pct == pytest.approx(plan.get("st06").target_valve_pct)


def test_actuator_bus_rejects_concurrent_grab():
    import time

    bus = TrimActuatorBus()
    entered = threading.Event()
    release = threading.Event()

    def hold():
        def slow():
            entered.set()
            release.wait(timeout=2.0)

        bus.run_serially("a", slow)

    holder = threading.Thread(target=hold)
    holder.start()
    assert entered.wait(timeout=2.0)
    rejected = False
    try:
        bus.run_serially("b", lambda: None)
    except RuntimeError:
        rejected = True
    finally:
        release.set()
        holder.join(timeout=2.0)
    assert rejected is True
    assert bus.active() is None


def test_service_persists_plan_run_and_audit(tmp_path):
    from heatnet.eventbus import EventBus
    from heatnet.store.jsonfile import JsonStore

    bus = EventBus()
    service = TrimService(JsonStore(str(tmp_path)), _driver_for(DEMO_READINGS), bus=bus)
    plan_view = service.plan(DEMO_READINGS)
    assert plan_view["method"] == "EQUAL_DELTA_T"
    assert any(line["station"] == "st06" for line in plan_view["deviations"])
    driver = service.driver
    driver.fail_readback("st05")
    run_view = service.execute(DEMO_READINGS)
    assert run_view["status"] == STATUS_PARTIAL
    assert run_view["fingerprint"] == run_view["plan"]["fingerprint"]
    assert run_view["reconciled"] is True
    actions = [event.action for event in bus.history()]
    assert "trim_planned" in actions
    assert "trim_executed" in actions
    # 方案与结果成对落盘，指纹互相咬合。
    plan_doc = tmp_path.glob("trim/plan/*.json")
    run_doc = tmp_path.glob("trim/run/*.json")
    assert list(plan_doc) and list(run_doc)


def test_platform_trim_endpoints_and_status(platform):
    api = Api(platform)
    status, plan = api.handle("POST", "/api/trim/plan", {}, {})
    assert status == 200
    assert plan["priority"] == "FAR_FIRST"
    assert plan["missing_stations"] == ["st03"]
    status, run = api.handle("POST", "/api/trim/run", {}, {"readback_fail": ["st02"]})
    assert status == 200
    assert run["status"] == "partial_failure"
    assert run["readback_missing_stations"] == ["st02"]
    status, body = api.handle("GET", "/api/status", {}, {})
    assert status == 200
    assert body["trim"]["method"] == "EQUAL_DELTA_T"
    assert body["trim"]["executed"] >= 1
