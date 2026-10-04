"""二次网等温差配平：方案、排序、执行、部分失败与三方一致。"""

from __future__ import annotations

from dataclasses import replace

import pytest

from heatnet.app import build_platform
from heatnet.balancing import (
    Direction,
    StationReading,
    StationStatus,
    assert_actuator_exclusion,
    compute_plan,
    execute_plan,
    schedule_actions,
    verify_consistency,
)
from heatnet.balancing.model import ValveAction
from heatnet.balancing.plan import METHOD
from heatnet.clock import ManualClock
from heatnet.config import default_config
from heatnet.console.api import Api
from heatnet.errors import InterlockError, StorageError
from heatnet.store.jsonfile import JsonStore
from heatnet.store.state import DurableState
from heatnet.valve.valve import Valve


def _limits():
    return default_config().balancing


def _reading(station, order, supply, ret, flow, valve=50.0, actuator=""):
    return StationReading(
        station=station,
        order=order,
        supply_c=supply,
        return_c=ret,
        flow_t_h=flow,
        valve_pct=valve,
        actuator=actuator,
    )


def _readings():
    # 近端（st01）贪水：流量大、温差小；远端（st05）欠水：流量小、温差大。
    return [
        _reading("st01", 1, 68.0, 58.0, 150.0),
        _reading("st02", 2, 68.0, 55.0, 120.0),
        _reading("st03", 3, 68.0, 52.0, 100.0),
        _reading("st04", 4, 68.0, 48.0, 80.0),
        _reading("st05", 5, 68.0, 44.0, 60.0),
    ]


def _valves(names=("st01", "st02", "st03", "st04", "st05"), position=50.0):
    return {name: Valve(f"{name}.mix", default_config().valve, position) for name in names}


def test_method_is_fixed_equal_delta_t():
    plan = compute_plan(_readings(), _limits(), "plan-1", 1000.0)
    assert METHOD == "EQUAL_DELTA_T"
    assert plan.method == "EQUAL_DELTA_T"
    assert plan.as_dict()["method"] == "EQUAL_DELTA_T"


def test_target_delta_is_flow_weighted_and_total_flow_conserved():
    plan = compute_plan(_readings(), _limits(), "plan-1", 1000.0)
    # ΔT* = Σ(G·ΔT)/ΣG = 7700/510 ≈ 15.10℃
    assert plan.target_delta_c == pytest.approx(7700.0 / 510.0)
    assert plan.total_flow_before_t_h == pytest.approx(510.0)
    # 总流量守恒是硬约束：目标流量之和等于当前总流量。
    assert plan.total_flow_after_t_h == pytest.approx(510.0)
    assert sum(a.target_flow_t_h for a in plan.actions) == pytest.approx(510.0)


def test_near_end_closes_far_end_opens():
    plan = compute_plan(_readings(), _limits(), "plan-1", 1000.0)
    by_station = {a.station: a for a in plan.actions}
    assert by_station["st01"].direction is Direction.CLOSE
    assert by_station["st02"].direction is Direction.CLOSE
    assert by_station["st03"].direction is Direction.OPEN
    assert by_station["st05"].direction is Direction.OPEN
    assert by_station["st01"].target_valve_pct < 50.0 < by_station["st05"].target_valve_pct
    # 每站偏差：负=贪水（近端），正=欠水（远端）。
    assert by_station["st01"].deviation_c < 0 < by_station["st05"].deviation_c
    assert by_station["st01"].deviation_c == pytest.approx(10.0 - plan.target_delta_c)


def test_step_is_capped_and_valve_clamped():
    readings = [_reading("st01", 1, 68.0, 58.0, 150.0), _reading("st05", 5, 68.0, 40.0, 60.0, valve=88.0)]
    plan = compute_plan(readings, _limits(), "plan-1", 1000.0)
    far = next(a for a in plan.actions if a.station == "st05")
    # 单周期步幅限幅 10%，阀位上限 90%：88 + 10 = 98 → 夹到 90。
    assert far.target_valve_pct == pytest.approx(90.0)
    assert far.status is StationStatus.CLAMPED


def test_valve_at_limit_holds_instead_of_reversing():
    # 阀位 5%（低于配平下限 10%）的近端站需要关小：限幅不能把动作顶反。
    readings = [
        _reading("st01", 1, 68.0, 58.0, 150.0, valve=5.0),
        _reading("st05", 5, 68.0, 44.0, 60.0),
    ]
    plan = compute_plan(readings, _limits(), "plan-1", 1000.0)
    assert all(a.station != "st01" for a in plan.actions)
    held = next(s for s in plan.skipped if s.station == "st01")
    assert held.status is StationStatus.AT_LIMIT


def test_deadband_holds_station():
    readings = [_reading(f"st{i:02d}", i, 68.0, 53.0, 100.0) for i in range(1, 4)]
    plan = compute_plan(readings, _limits(), "plan-1", 1000.0)
    assert plan.status == "NO_ACTION"
    assert plan.actions == ()
    assert all(s.status is StationStatus.HOLD_DEADBAND for s in plan.skipped)


def test_missing_readback_is_partial_failure_and_excluded():
    readings = _readings()
    readings[2] = _reading("st03", 3, None, None, None)
    plan = compute_plan(readings, _limits(), "plan-1", 1000.0)
    assert plan.status == "PARTIAL"
    skipped = next(s for s in plan.skipped if s.station == "st03")
    assert skipped.status is StationStatus.SKIPPED_NO_READBACK
    # 回读缺失的站不参与目标温差计算，也不下发动作。
    assert plan.target_delta_c == pytest.approx(6100.0 / 410.0)
    assert all(a.station != "st03" for a in plan.actions)
    assert len(plan.actions) == 4


def test_invalid_readback_is_skipped():
    readings = _readings()
    readings[0] = _reading("st01", 1, 50.0, 55.0, 150.0)  # 回温高于供温
    plan = compute_plan(readings, _limits(), "plan-1", 1000.0)
    skipped = next(s for s in plan.skipped if s.station == "st01")
    assert skipped.status is StationStatus.SKIPPED_INVALID
    assert plan.status == "PARTIAL"


def test_all_missing_readback_yields_no_actions():
    readings = [_reading(f"st{i:02d}", i, None, None, None) for i in range(1, 4)]
    plan = compute_plan(readings, _limits(), "plan-1", 1000.0)
    assert plan.status == "ALL_SKIPPED"
    assert plan.actions == ()
    assert plan.ok() is False


def test_schedule_orders_opens_far_first_then_closes_near_first():
    plan = compute_plan(_readings(), _limits(), "plan-1", 1000.0)
    actions = list(plan.actions)
    assert [a.sequence for a in actions] == list(range(1, len(actions) + 1))
    phases = [a.phase for a in actions]
    assert phases == sorted(phases)  # 开阀阶段全部在关阀阶段之前
    opens = [a.station for a in actions if a.direction is Direction.OPEN]
    closes = [a.station for a in actions if a.direction is Direction.CLOSE]
    assert opens == ["st05", "st04", "st03"]  # 远端优先
    assert closes == ["st01", "st02"]  # 近端先压


def test_actuator_exclusion_holds_and_validator_catches_race():
    readings = [
        _reading("st01", 1, 68.0, 58.0, 150.0, actuator="ACT-1"),
        _reading("st05", 5, 68.0, 44.0, 60.0, actuator="ACT-1"),
    ]
    plan = compute_plan(readings, _limits(), "plan-1", 1000.0)
    sequences = [a.sequence for a in plan.actions]
    assert len(sequences) == len(set(sequences))  # 同一执行器不会被同时抢占
    # 人为构造两台阀同拍抢一个执行器，校验必须拦下。
    broken = tuple(replace(a, sequence=1) for a in plan.actions)
    with pytest.raises(InterlockError):
        assert_actuator_exclusion(broken)


def test_execute_moves_valves_and_matches_plan(tmp_path):
    plan = compute_plan(_readings(), _limits(), "plan-1", 1000.0)
    valves = _valves()
    durable = DurableState(JsonStore(str(tmp_path)))
    execution = execute_plan(plan, valves, durable)
    assert execution.ok() is True
    by_station = {a.station: a for a in plan.actions}
    for name, valve in valves.items():
        assert valve.position_pct() == pytest.approx(by_station[name].target_valve_pct)
        assert durable.load(f"valve/{name}.mix")["position_pct"] == pytest.approx(
            by_station[name].target_valve_pct
        )
    assert verify_consistency(plan, execution) == []


class _FlakyDurable:
    """指定组件写盘失败，其余正常落盘。"""

    def __init__(self, inner: DurableState, fail_on: str) -> None:
        self._inner = inner
        self._fail_on = fail_on

    def load(self, component: str) -> dict:
        return self._inner.load(component)

    def save(self, component: str, payload: object) -> str:
        if self._fail_on in component:
            raise StorageError("模拟写盘失败", component="store")
        return self._inner.save(component, payload)


def test_execute_continues_after_single_valve_failure(tmp_path):
    plan = compute_plan(_readings(), _limits(), "plan-1", 1000.0)
    valves = _valves()
    durable = _FlakyDurable(DurableState(JsonStore(str(tmp_path))), fail_on="st03")
    execution = execute_plan(plan, valves, durable)
    assert execution.ok() is False
    by_station = {a.station: a for a in execution.actions}
    assert by_station["st03"].result == "FAILED"
    assert valves["st03"].position_pct() == pytest.approx(50.0)  # 失败站保持原阀位
    done = [a for a in execution.actions if a.result == "DONE"]
    assert len(done) == 4  # 其余站继续
    # 失败也是如实记录的部分失败：方案与执行仍然一致。
    assert verify_consistency(plan, execution) == []


def test_verify_detects_tampered_execution(tmp_path):
    plan = compute_plan(_readings(), _limits(), "plan-1", 1000.0)
    valves = _valves()
    execution = execute_plan(plan, valves, DurableState(JsonStore(str(tmp_path))))
    assert verify_consistency(plan, execution) == []
    tampered_action = replace(execution.actions[0], actual_valve_pct=1.0)
    tampered = replace(execution, actions=(tampered_action, *execution.actions[1:]))
    assert verify_consistency(plan, tampered)
    wrong_plan = replace(execution, plan_id="plan-2")
    assert verify_consistency(plan, wrong_plan)


def test_skipped_station_never_acts(tmp_path):
    readings = _readings()
    readings[3] = _reading("st04", 4, None, None, None)
    plan = compute_plan(readings, _limits(), "plan-1", 1000.0)
    valves = _valves()
    execution = execute_plan(plan, valves, DurableState(JsonStore(str(tmp_path))))
    assert valves["st04"].position_pct() == pytest.approx(50.0)
    assert verify_consistency(plan, execution) == []


def _platform_readings():
    return [
        {"station": "st01", "supply_c": 68.0, "return_c": 57.0, "flow_t_h": 150.0},
        {"station": "st02", "supply_c": 68.0, "return_c": 54.0, "flow_t_h": 120.0},
        {"station": "st03", "supply_c": 68.0, "return_c": 51.0, "flow_t_h": 100.0},
        {"station": "st04", "supply_c": 68.0, "return_c": None, "flow_t_h": None},
        {"station": "st05", "supply_c": 68.0, "return_c": 45.0, "flow_t_h": 65.0},
        {"station": "st06", "supply_c": 68.0, "return_c": 42.0, "flow_t_h": 50.0},
    ]


def test_platform_balancing_end_to_end(platform):
    for valve in platform.valves.values():
        valve.apply(50.0)
    report = platform.balance_secondary(_platform_readings(), "tester")
    assert report["status"] == "PARTIAL"
    assert report["consistent"] is True
    assert report["problems"] == []
    rows = {row["station"]: row for row in report["stations"]}
    assert rows["st01"]["direction"] == "CLOSE"
    assert rows["st06"]["direction"] == "OPEN"
    assert rows["st04"]["status"] == "SKIPPED_NO_READBACK"
    assert rows["st04"]["target_valve_pct"] == pytest.approx(50.0)
    # 方案、实际阀位一致；跳过站的阀没动。
    assert platform.valves["st01"].position_pct() == pytest.approx(rows["st01"]["target_valve_pct"])
    assert platform.valves["st04"].position_pct() == pytest.approx(50.0)
    # 审计里有方案、跳过与逐台动作，且与控制台展示同源。
    entries = [e for e in platform.audit.entries(limit=300) if e["component"] == "balancing"]
    assert any(e["action"] == "plan" and e["payload"]["plan_id"] == report["plan_id"] for e in entries)
    assert any(e["action"] == "station_skipped" and e["asset"] == "st04" for e in entries)
    moves = [e for e in entries if e["action"] == "valve_move"]
    assert len(moves) == report["actions"]
    assert platform.audit.alarms()
    # 控制台查询与方案落盘同源。
    stored = platform.durable.load("balancing/last")
    assert stored["plan_id"] == report["plan_id"]
    assert platform.balancing_report()["plan"]["plan_id"] == report["plan_id"]


def test_balancing_report_survives_restart(tmp_path):
    clock = ManualClock()
    root = str(tmp_path / "state")
    platform = build_platform(default_config(), clock=clock, storage_root=root)
    for valve in platform.valves.values():
        valve.apply(50.0)
    report = platform.balance_secondary(_platform_readings(), "tester")
    restarted = build_platform(default_config(), clock=clock, storage_root=root)
    restored = restarted.balancing_report()["plan"]
    assert restored is not None
    assert restored["plan_id"] == report["plan_id"]
    assert restored["stations"] == report["stations"]


def test_console_balancing_endpoints(platform):
    api = Api(platform)
    for valve in platform.valves.values():
        valve.apply(50.0)
    status, payload = api.handle("POST", "/api/balancing", {}, {"readings": _platform_readings()})
    assert status == 200
    assert payload["consistent"] is True
    status, got = api.handle("GET", "/api/balancing", {}, {})
    assert status == 200
    assert got["plan"]["plan_id"] == payload["plan_id"]
    status, _ = api.handle("POST", "/api/balancing", {}, {})
    assert status == 400
    status, _ = api.handle(
        "POST", "/api/balancing", {}, {"readings": [{"station": "st99", "supply_c": 60.0}]}
    )
    assert status == 409


def test_unknown_or_duplicate_station_rejected(platform):
    with pytest.raises(Exception):
        platform.balance_secondary([{"station": "st99", "supply_c": 60.0}])
    with pytest.raises(ValueError):
        platform.balance_secondary(
            [
                {"station": "st01", "supply_c": 68.0, "return_c": 57.0, "flow_t_h": 150.0},
                {"station": "st01", "supply_c": 68.0, "return_c": 57.0, "flow_t_h": 150.0},
            ]
        )


def test_schedule_actions_is_pure_sorting():
    plan = compute_plan(_readings(), _limits(), "plan-1", 1000.0)
    again = schedule_actions(tuple(replace(a, sequence=0, phase=0) for a in plan.actions))
    assert [a.station for a in again] == [a.station for a in plan.actions]
