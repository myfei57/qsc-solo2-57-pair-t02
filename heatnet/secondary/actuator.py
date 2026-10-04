"""配平阀串行执行器。

方案在 :mod:`heatnet.secondary.trim` 里一次性算好并盖了指纹；这里只负责按方案
动阀，三条纪律贯穿到底：

* **一个执行器**。全网配平阀共用同一个 :class:`TrimActuatorBus`（内部一把
  锁），任何时刻只允许一只阀在动作，不允许几台阀并抢；动作顺序固定按站序
  远端在前、近端兜底，和"保远端优先"的裁决顺序一致。
* **回读缺失＝部分失败**。指令发出后读不回阀位，该站记
  ``readback_missing``，阀位按"不动"处理（内存与落盘都保持原值），其余站
  照常继续，整组结果只降级为 ``partial_failure``，不整体失败。
* **方案、实际阀位、审计同一口径**。每站动作记一条流水，整组结束做一次
  对账：参与站实际阀位必须逐站等于方案目标，缺测/回读缺失站必须等于动作
  前阀位；对得上才 ``reconciled``。审计里同时带方案指纹，防止执行的和展示
  的不是同一份方案。
"""

from __future__ import annotations

import threading
from dataclasses import dataclass

from .trim import (
    ROW_READBACK_MISSING,
    ROW_TELEMETRY_MISSING,
    ROW_UNCHANGED,
    STATUS_INFEASIBLE,
    STATUS_OK,
    STATUS_PARTIAL,
    TrimPlan,
)

# 阀位回读到站后的到位公差（个百分点）。
SETTLE_BAND_PCT = 1.0
# 回读不符时允许一次重试（重新下同一目标）；再不符按未到位如实记账，不盲动。
MAX_RETRIES = 1


class TrimDriver:
    """配平阀驱动接口：下单后必须回读，回读缺失返回 ``None``。"""

    def move(self, station: str, target_pct: float) -> float | None:
        """把某站配平阀送到目标开度，返回实际回读；回读缺失返回 ``None``。"""

        raise NotImplementedError

    def position(self, station: str) -> float | None:
        """读取某站当前阀位，回读缺失返回 ``None``。"""

        raise NotImplementedError


class SimulatedTrimDriver(TrimDriver):
    """内存模拟驱动：指令即到位并回读，可按站注入回读缺失。

    被注入故障的站在下单时回读 ``None``，且内存阀位保持原值——模拟
    "阀动了但读不回"时按"没动"处理的现场纪律。这里保持纯模拟，阀位当前态
    是否落盘由上层编排决定。
    """

    def __init__(self, positions: dict[str, float] | None = None) -> None:
        self._positions: dict[str, float] = dict(positions or {})
        self._missing: set[str] = set()
        self._commands: list[tuple[str, float]] = []

    def seed(self, station: str, position_pct: float) -> None:
        """登记一站当前阀位。"""

        self._positions[station] = max(0.0, min(100.0, float(position_pct)))

    def fail_readback(self, station: str) -> None:
        """让某站在消缺前的所有动作回读缺失。"""

        self._missing.add(station)

    def commands(self) -> list[tuple[str, float]]:
        """返回已下发指令（验证串行顺序用）。"""

        return list(self._commands)

    def move(self, station: str, target_pct: float) -> float | None:
        self._commands.append((station, float(target_pct)))
        if station in self._missing:
            # 持续回读缺失：故障未消除前重发也读不回，阀位保持原值，按"不动"记账。
            return None
        self._positions[station] = max(0.0, min(100.0, float(target_pct)))
        return self._positions[station]

    def clear_fault(self, station: str) -> None:
        """现场消缺后解除某站的回读缺失注入。"""

        self._missing.discard(station)

    def position(self, station: str) -> float | None:
        if station in self._missing:
            return None
        return self._positions.get(station)


@dataclass(frozen=True)
class TrimStationResult:
    """一个站执行后的对账行。"""

    station: str
    order: int
    status: str
    planned_valve_pct: float | None
    actual_valve_pct: float | None
    valve_error_pct: float | None
    moved: bool
    retries: int

    def as_dict(self) -> dict:
        return {
            "station": self.station,
            "order": self.order,
            "status": self.status,
            "planned_valve_pct": _round(self.planned_valve_pct),
            "actual_valve_pct": _round(self.actual_valve_pct),
            "valve_error_pct": _round(self.valve_error_pct),
            "moved": self.moved,
            "retries": self.retries,
        }


@dataclass(frozen=True)
class TrimRun:
    """一次配平执行的完整结果（落审计的就是这份）。"""

    status: str
    method: str
    priority: str
    fingerprint: str
    acted_count: int
    readback_missing_stations: tuple[str, ...]
    telemetry_missing_stations: tuple[str, ...]
    reconciled: bool
    steps: tuple[dict, ...]
    results: tuple[TrimStationResult, ...]

    def as_dict(self) -> dict:
        return {
            "status": self.status,
            "method": self.method,
            "priority": self.priority,
            "fingerprint": self.fingerprint,
            "acted_count": self.acted_count,
            "readback_missing_stations": list(self.readback_missing_stations),
            "telemetry_missing_stations": list(self.telemetry_missing_stations),
            "reconciled": self.reconciled,
            "steps": list(self.steps),
            "results": [row.as_dict() for row in self.results],
        }


class TrimActuatorBus:
    """配平阀共享执行器：一把锁串行化全部阀位动作。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._active: str | None = None

    def run_serially(self, station: str, action):
        """占有执行器跑完一站动作；执行器被占时立即拒绝，绝不排队并抢。

        用非阻塞抢锁：阀门动作（含等回读）期间锁一直不释放，任何别的站
        此刻进来都直接抛错，而不是排在后面等前一只阀动作完再悄悄执行。
        """

        if not self._lock.acquire(blocking=False):
            raise RuntimeError(f"执行器正被 {self._active} 占用，{station} 不得并抢")
        self._active = station
        try:
            if self.active() != station:
                raise RuntimeError("执行器占用站与动作站不一致")
            return action()
        finally:
            self._active = None
            self._lock.release()

    def active(self) -> str | None:
        """返回当前占用执行器的站（空闲为 ``None``）。

        动作闭包在持锁期间也要调用它自检，因此这里不再抢锁；``_active``
        只在 ``run_serially`` 的临界区内改写，读到的至多是上一拍的值。
        """

        return self._active


def execute_trim(
    plan: TrimPlan,
    driver: TrimDriver,
    bus: TrimActuatorBus | None = None,
) -> TrimRun:
    """按方案远端优先串行动阀，返回逐站对账结果。

    缺测站不下发任何指令；目标与当前阀位相同的站记 ``unchanged`` 也不动作。
    方案本身不可行（远端被钉到流量下沿仍兜不住总水量）时照常下发可执行
    部分，结果状态带 ``infeasible`` 痕迹，由审计与控制台提示再配一轮。
    """

    bus = bus or TrimActuatorBus()
    steps: list[dict] = []
    results: list[TrimStationResult] = []
    readback_missing: list[str] = []
    telemetry_missing: list[str] = list(plan.missing_stations)
    acted = 0

    # 不动的站先登记：缺测站不下发任何指令；目标与现状一致的站也不占执行器。
    action_rows = list(plan.action_rows())
    moving = {row.station for row in action_rows}
    for row in plan.rows:
        if row.station in moving:
            continue
        if row.status == ROW_TELEMETRY_MISSING:
            results.append(
                TrimStationResult(row.station, row.order, ROW_TELEMETRY_MISSING, None, None, None, False, 0)
            )
        else:
            results.append(
                TrimStationResult(row.station, row.order, ROW_UNCHANGED, row.valve_pct, row.valve_pct, 0.0, False, 0)
            )

    # 动阀顺序与保端裁决一致：远端在前、近端兜底，全程串行不并抢。
    for row in action_rows:
        before = row.valve_pct

        def _act() -> tuple[float | None, int]:
            actual = driver.move(row.station, row.target_valve_pct)
            retries = 0
            while actual is None and retries < MAX_RETRIES:
                retries += 1
                actual = driver.move(row.station, row.target_valve_pct)
            return actual, retries

        actual, retries = bus.run_serially(row.station, _act)
        holder = row.station
        acted += 1
        if actual is None:
            readback_missing.append(row.station)
            steps.append(
                {
                    "seq": len(steps) + 1,
                    "station": row.station,
                    "order": row.order,
                    "holder": holder,
                    "target_pct": round(row.target_valve_pct, 2),
                    "readback": None,
                    "held_pct": round(before, 2),
                    "retries": retries,
                }
            )
            results.append(
                TrimStationResult(row.station, row.order, ROW_READBACK_MISSING, row.target_valve_pct, before, None, False, retries)
            )
            continue

        error = float(actual) - row.target_valve_pct
        steps.append(
            {
                "seq": len(steps) + 1,
                "station": row.station,
                "order": row.order,
                "holder": holder,
                "target_pct": round(row.target_valve_pct, 2),
                "readback": round(float(actual), 2),
                "error_pct": round(error, 2),
                "retries": retries,
            }
        )
        results.append(
            TrimStationResult(
                row.station,
                row.order,
                row.status,
                row.target_valve_pct,
                float(actual),
                error,
                True,
                retries,
            )
        )

    results.sort(key=lambda item: item.order)
    reconciled = _reconcile(plan, results)
    if readback_missing:
        status = STATUS_PARTIAL
    elif not plan.feasible:
        status = STATUS_INFEASIBLE
    else:
        status = STATUS_OK
    return TrimRun(
        status=status,
        method=plan.method,
        priority=plan.priority,
        fingerprint=plan.fingerprint,
        acted_count=acted,
        readback_missing_stations=tuple(readback_missing),
        telemetry_missing_stations=tuple(telemetry_missing),
        reconciled=reconciled,
        steps=tuple(steps),
        results=tuple(results),
    )


def _reconcile(plan: TrimPlan, results: list[TrimStationResult]) -> bool:
    """逐站对账：动作站实际阀位必须等于方案目标（公差内），其余站必须没动。"""

    by_station = {item.station: item for item in results}
    for row in plan.rows:
        item = by_station.get(row.station)
        if item is None:
            return False
        if row.status == ROW_TELEMETRY_MISSING:
            # 缺测站本就没有可信阀位回读，唯一要求是它没被动过。
            if item.moved or item.status != ROW_TELEMETRY_MISSING:
                return False
            continue
        if item.actual_valve_pct is None:
            return False
        if item.status == ROW_READBACK_MISSING:
            # 回读缺失按"不动"处理：实际阀位必须保持动作前值，才算账实一致。
            if abs(item.actual_valve_pct - (row.valve_pct or 0.0)) > 1e-9:
                return False
            continue
        target = row.target_valve_pct
        if target is None or abs((row.valve_move_pct or 0.0)) <= 1e-9:
            if abs(item.actual_valve_pct - (row.valve_pct or 0.0)) > 1e-9:
                return False
            continue
        if abs(item.actual_valve_pct - target) > SETTLE_BAND_PCT:
            return False
    return True


def _round(value: float | None, digits: int = 2) -> float | None:
    return None if value is None else round(float(value), digits)
