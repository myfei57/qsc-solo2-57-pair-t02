"""按配平方案逐台动阀。

严格按全局序号一台一台执行，同一执行器不会被两台阀同时占用。每台阀沿用
"先落盘、再改内存"的动作口径；某一台写盘失败只记该站失败、保持原阀位，
其余站继续——这与回读缺失同属部分失败，方案不整体作废。
"""

from __future__ import annotations

from dataclasses import dataclass

from ..errors import StorageError
from ..store.state import DurableState
from ..valve.move import move_valve
from ..valve.valve import Valve
from .model import BalancingPlan


@dataclass(frozen=True)
class ExecutedAction:
    """一台阀的执行结果（实际阀位以阀门回读为准）。"""

    action_id: str
    station: str
    sequence: int
    phase: int
    target_valve_pct: float
    actual_valve_pct: float
    result: str  # DONE / FAILED

    def as_dict(self) -> dict:
        return {
            "action_id": self.action_id,
            "station": self.station,
            "sequence": self.sequence,
            "phase": self.phase,
            "target_valve_pct": round(self.target_valve_pct, 2),
            "actual_valve_pct": round(self.actual_valve_pct, 2),
            "result": self.result,
        }


@dataclass(frozen=True)
class ExecutionReport:
    """一轮配平的执行报告。"""

    plan_id: str
    actions: tuple[ExecutedAction, ...]

    def ok(self) -> bool:
        """是否全部动作都执行成功。"""

        return all(action.result == "DONE" for action in self.actions)

    def as_dict(self) -> dict:
        return {
            "plan_id": self.plan_id,
            "ok": self.ok(),
            "actions": [action.as_dict() for action in self.actions],
        }


def execute_plan(
    plan: BalancingPlan,
    valves: dict[str, Valve],
    durable: DurableState,
) -> ExecutionReport:
    """按全局序号逐台执行方案动作；单台失败不阻断其余站。"""

    executed: list[ExecutedAction] = []
    for action in sorted(plan.actions, key=lambda item: item.sequence):
        valve = valves[action.station]
        try:
            move = move_valve(valve, durable, action.target_valve_pct)
            executed.append(
                ExecutedAction(
                    action_id=action.action_id,
                    station=action.station,
                    sequence=action.sequence,
                    phase=action.phase,
                    target_valve_pct=action.target_valve_pct,
                    actual_valve_pct=move.position_pct,
                    result="DONE",
                )
            )
        except StorageError:
            # 部分失败：该站保持原阀位，其余站继续。
            executed.append(
                ExecutedAction(
                    action_id=action.action_id,
                    station=action.station,
                    sequence=action.sequence,
                    phase=action.phase,
                    target_valve_pct=action.target_valve_pct,
                    actual_valve_pct=valve.position_pct(),
                    result="FAILED",
                )
            )
    return ExecutionReport(plan_id=plan.plan_id, actions=tuple(executed))
