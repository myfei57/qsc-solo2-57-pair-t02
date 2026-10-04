"""配平数据模型。

方案里的每一个数字都要同时出现在三处：控制台展示、阀门执行、审计流水。
模型只定义一次、三处共用，避免"方案写的、阀门动的、审计记的"对不上。
"""

from __future__ import annotations

import enum
from dataclasses import dataclass


class Direction(enum.Enum):
    """阀位动作方向。"""

    OPEN = "OPEN"  # 开大——抬远端
    CLOSE = "CLOSE"  # 关小——压近端


class StationStatus(enum.Enum):
    """单站配平状态。"""

    ACTION = "ACTION"  # 生成阀位动作
    CLAMPED = "CLAMPED"  # 目标阀位触及限幅，按限幅执行
    HOLD_DEADBAND = "HOLD_DEADBAND"  # 温差偏差在死区内，不动作
    AT_LIMIT = "AT_LIMIT"  # 已在限位，朝需要的方向动不了
    SKIPPED_NO_READBACK = "SKIPPED_NO_READBACK"  # 回读缺失：部分失败，按不动处理
    SKIPPED_INVALID = "SKIPPED_INVALID"  # 回读非法（温差或流量不为正）


# 这两类跳过属于部分失败：该站按不动处理，其余站继续。
FAILURE_STATUSES = (StationStatus.SKIPPED_NO_READBACK, StationStatus.SKIPPED_INVALID)


def _r2(value: float | None) -> float | None:
    """保留两位小数；``None``（回读缺失）原样保留。"""

    return None if value is None else round(float(value), 2)


@dataclass(frozen=True)
class StationReading:
    """一个站的二次侧回读；``None`` 表示该路回读缺失。"""

    station: str
    order: int  # 站序：距热源由近及远，1 = 最近端
    supply_c: float | None
    return_c: float | None  # None => 回读缺失
    flow_t_h: float | None  # None => 回读缺失
    valve_pct: float  # 当前阀位 %
    actuator: str = ""  # 执行器编号；空表示每站一台

    def actuator_id(self) -> str:
        """返回执行器编号（缺省每站一台）。"""

        return self.actuator or f"{self.station}.actuator"

    def readback_missing(self) -> bool:
        """关键回读（供温/回温/流量）是否缺失。"""

        return self.supply_c is None or self.return_c is None or self.flow_t_h is None

    def delta_c(self) -> float | None:
        """供回水温差；回读缺失时为 ``None``。"""

        if self.supply_c is None or self.return_c is None:
            return None
        return float(self.supply_c) - float(self.return_c)


@dataclass(frozen=True)
class ValveAction:
    """一台阀的配平动作（含执行排序）。"""

    plan_id: str
    station: str
    order: int
    actuator: str
    direction: Direction
    supply_c: float
    return_c: float
    delta_c: float
    deviation_c: float  # 温差偏差 ΔTᵢ−ΔT*：负=贪水要压，正=欠水要抬
    flow_t_h: float
    target_flow_t_h: float
    valve_pct: float
    target_valve_pct: float
    status: StationStatus = StationStatus.ACTION
    action_id: str = ""
    phase: int = 0  # 1=开阀阶段 2=关阀阶段
    sequence: int = 0  # 全局执行序号（全序）

    def as_dict(self) -> dict:
        return {
            "action_id": self.action_id,
            "plan_id": self.plan_id,
            "station": self.station,
            "order": self.order,
            "actuator": self.actuator,
            "direction": self.direction.value,
            "supply_c": round(self.supply_c, 2),
            "return_c": round(self.return_c, 2),
            "delta_c": round(self.delta_c, 2),
            "deviation_c": round(self.deviation_c, 2),
            "flow_t_h": round(self.flow_t_h, 2),
            "target_flow_t_h": round(self.target_flow_t_h, 2),
            "valve_pct": round(self.valve_pct, 2),
            "target_valve_pct": round(self.target_valve_pct, 2),
            "status": self.status.value,
            "phase": self.phase,
            "sequence": self.sequence,
        }


@dataclass(frozen=True)
class SkippedStation:
    """一个本轮不动作的站（回读缺失、回读非法、死区或到限），保持原阀位。"""

    station: str
    order: int
    status: StationStatus
    reason: str
    valve_pct: float
    actuator: str = ""
    supply_c: float | None = None
    return_c: float | None = None
    flow_t_h: float | None = None
    delta_c: float | None = None
    deviation_c: float | None = None

    def as_dict(self) -> dict:
        return {
            "station": self.station,
            "order": self.order,
            "status": self.status.value,
            "reason": self.reason,
            "valve_pct": round(self.valve_pct, 2),
            "actuator": self.actuator,
            "supply_c": _r2(self.supply_c),
            "return_c": _r2(self.return_c),
            "flow_t_h": _r2(self.flow_t_h),
            "delta_c": _r2(self.delta_c),
            "deviation_c": _r2(self.deviation_c),
        }


@dataclass(frozen=True)
class BalancingPlan:
    """一轮等温差配平的完整方案。"""

    plan_id: str
    created_ts: float
    method: str
    target_delta_c: float
    total_flow_before_t_h: float
    total_flow_after_t_h: float
    status: str  # OK / PARTIAL / NO_ACTION / ALL_SKIPPED
    actions: tuple[ValveAction, ...]
    skipped: tuple[SkippedStation, ...]

    def ok(self) -> bool:
        """方案本身是否没有部分失败（NO_ACTION 也算方案成立）。"""

        return self.status in ("OK", "NO_ACTION")

    def partial(self) -> bool:
        """是否存在部分失败（有站因回读问题被跳过）。"""

        return self.status == "PARTIAL"

    def rows(self) -> list[dict]:
        """控制台展示：每站一行（动作站与跳过站都在内），按站序排列。"""

        rows: list[dict] = []
        for action in self.actions:
            rows.append({**action.as_dict(), "reason": ""})
        for skip in self.skipped:
            rows.append(
                {
                    "action_id": "",
                    "plan_id": self.plan_id,
                    "station": skip.station,
                    "order": skip.order,
                    "actuator": skip.actuator,
                    "direction": "HOLD",
                    "supply_c": _r2(skip.supply_c),
                    "return_c": _r2(skip.return_c),
                    "delta_c": _r2(skip.delta_c),
                    "deviation_c": _r2(skip.deviation_c),
                    "flow_t_h": _r2(skip.flow_t_h),
                    "target_flow_t_h": _r2(skip.flow_t_h),
                    "valve_pct": round(skip.valve_pct, 2),
                    "target_valve_pct": round(skip.valve_pct, 2),
                    "status": skip.status.value,
                    "phase": 0,
                    "sequence": 0,
                    "reason": skip.reason,
                }
            )
        return sorted(rows, key=lambda row: (row["order"], row["station"]))

    def as_dict(self) -> dict:
        return {
            "plan_id": self.plan_id,
            "created_ts": round(self.created_ts, 3),
            "method": self.method,
            "status": self.status,
            "ok": self.ok(),
            "target_delta_c": round(self.target_delta_c, 2),
            "total_flow_before_t_h": round(self.total_flow_before_t_h, 2),
            "total_flow_after_t_h": round(self.total_flow_after_t_h, 2),
            "actions": [action.as_dict() for action in self.actions],
            "skipped": [skip.as_dict() for skip in self.skipped],
            "stations": self.rows(),
        }
