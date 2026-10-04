"""二次侧配平编排：方案、执行、偏差与审计共用同一份口径。

值班员看到的是每个站三笔偏差：

* **温差偏差**＝本站当前温差−配平目标温差（正＝温差偏大、回水偏冷；
  负＝近端贪水、回水偏暖）；配平目标是全部参与站归零。
* **流量偏差**＝方案目标流量−当前流量（正＝需要抬水、远端；负＝压水、近端）。
* **阀位偏差**＝方案目标阀位−当前阀位；执行后再补一笔
  **回读偏差**＝实际回读阀位−方案目标阀位，用来核对"方案、实际阀位、
  审计"三者一致。

编排只做串联：回读 → 锁死规则算方案（:func:`build_trim_plan`）→ 远端优先
串行动阀（:func:`execute_trim`）→ 逐站偏差 → 方案与结果一起落盘并写审计。
审计流水里带方案指纹，方案文档与结果文档互相引用同一指纹，控制台展示直接
读这份对象，不另算第二份数。
"""

from __future__ import annotations

from ..eventbus import EventBus, Event
from ..ids import IdFactory
from ..store.jsonfile import JsonStore
from .actuator import TrimActuatorBus, TrimDriver, TrimRun, execute_trim
from .trim import ROW_TELEMETRY_MISSING, TrimPlan, build_trim_plan

# 落盘文档标识前缀（方案与结果成对）。
PLAN_PREFIX = "trim/plan"
RUN_PREFIX = "trim/run"


def station_deviations(plan: TrimPlan, run: TrimRun | None = None) -> list[dict]:
    """按站序返回每站偏差行；执行后 ``run`` 非空时追加实际阀位与回读偏差。"""

    actual_by_station = {item.station: item for item in run.results} if run is not None else {}
    lines: list[dict] = []
    for row in plan.rows:
        if row.status == ROW_TELEMETRY_MISSING:
            lines.append(
                {
                    "station": row.station,
                    "order": row.order,
                    "status": row.status,
                    "delta_c_error": None,
                    "return_c_error": None,
                    "flow_error_t_h": None,
                    "valve_error_pct": None,
                    "readback_error_pct": None,
                    "actual_valve_pct": None,
                }
            )
            continue
        item = actual_by_station.get(row.station)
        lines.append(
            {
                "station": row.station,
                "order": row.order,
                "status": item.status if item is not None else row.status,
                "delta_c_error": round((row.delta_c or 0.0) - plan.target_delta_c, 2),
                "return_c_error": round((row.return_c or 0.0) - (plan.target_return_c or 0.0), 2)
                if plan.target_return_c is not None
                else None,
                "flow_error_t_h": round((row.target_flow_t_h or 0.0) - (row.flow_t_h or 0.0), 2),
                "valve_error_pct": round((row.target_valve_pct or 0.0) - (row.valve_pct or 0.0), 2),
                "readback_error_pct": round(
                    (item.actual_valve_pct - (row.target_valve_pct or item.actual_valve_pct or 0.0)), 2
                )
                if item is not None and item.actual_valve_pct is not None
                else None,
                "actual_valve_pct": round(item.actual_valve_pct, 2)
                if item is not None and item.actual_valve_pct is not None
                else None,
            }
        )
    return lines


class TrimService:
    """配平编排服务：持有共享执行器，方案先落盘，执行结果与方案对账。"""

    def __init__(self, store: JsonStore, driver: TrimDriver, bus: EventBus | None = None) -> None:
        self._store = store
        self._driver = driver
        self._actuator = TrimActuatorBus()
        self._ids = IdFactory("trim")
        self._bus = bus

    @property
    def driver(self) -> TrimDriver:
        return self._driver

    def plan(self, readings: list[dict]) -> dict:
        """只算方案并落盘，不动任何阀（控制台先展示、值班员确认后再执行）。"""

        plan = build_trim_plan(readings)
        plan_id = self._ids.next()
        self._store.write(
            f"{PLAN_PREFIX}/{plan_id}",
            {"plan_id": plan_id, "method": plan.method, "priority": plan.priority, "plan": plan.as_dict()},
        )
        self._last_plan_id = plan_id
        self._publish(
            "trim_planned",
            "notice",
            "二次侧配平方案已生成",
            plan_id=plan_id,
            fingerprint=plan.fingerprint,
            method=plan.method,
            priority=plan.priority,
            participant_count=plan.participant_count,
            missing=list(plan.missing_stations),
            target_delta_c=round(plan.target_delta_c, 2),
        )
        return {"plan_id": plan_id, **plan.as_dict(), "deviations": station_deviations(plan)}

    def execute(self, readings: list[dict] | None = None, plan_id: str | None = None) -> dict:
        """按方案执行：缺省重算并落盘一份新方案，再远端优先串行动阀。"""

        if plan_id is not None:
            stored = self._store.read(f"{PLAN_PREFIX}/{plan_id}")
            if not isinstance(stored, dict):
                raise KeyError(f"配平方案不存在: {plan_id}")
            plan = build_trim_plan(_readings_from_plan(stored["plan"]))
            if plan.fingerprint != stored["plan"]["fingerprint"]:
                raise ValueError("落盘方案与重算指纹不一致，拒绝执行")
        else:
            if readings is None:
                raise ValueError("执行配平必须提供回读或指定方案")
            planned = self.plan(readings)
            plan_id = planned["plan_id"]
            plan = build_trim_plan(readings)

        run = execute_trim(plan, self._driver, self._actuator)
        run_id = self._ids.next()
        self._store.write(
            f"{RUN_PREFIX}/{run_id}",
            {
                "run_id": run_id,
                "plan_id": plan_id,
                "fingerprint": run.fingerprint,
                "run": run.as_dict(),
            },
        )
        deviations = station_deviations(plan, run)
        severity = "notice" if run.status == "ok" else "warning"
        self._publish(
            "trim_executed",
            severity,
            "二次侧配平执行完成" if run.status == "ok" else f"二次侧配平部分失败：{run.status}",
            plan_id=plan_id,
            run_id=run_id,
            fingerprint=run.fingerprint,
            status=run.status,
            reconciled=run.reconciled,
            acted=run.acted_count,
            readback_missing=list(run.readback_missing_stations),
            telemetry_missing=list(run.telemetry_missing_stations),
        )
        return {
            "plan_id": plan_id,
            "run_id": run_id,
            **run.as_dict(),
            "plan": plan.as_dict(),
            "deviations": deviations,
        }

    def _publish(self, action: str, severity: str, message: str, **payload) -> None:
        if self._bus is None:
            return
        self._bus.publish(Event.of("secondary", "secondary", action, severity=severity, message=message, **payload))


def _readings_from_plan(plan_payload: dict) -> list[dict]:
    """从方案快照还原执行所需回读（方案自包含，执行不依赖当拍回读）。"""

    readings: list[dict] = []
    for row in plan_payload["rows"]:
        readings.append(
            {
                "station": row["station"],
                "supply_c": row["supply_c"],
                "return_c": row["return_c"],
                "flow_t_h": row["flow_t_h"],
                "valve_pct": row["valve_pct"],
            }
        )
    return readings
