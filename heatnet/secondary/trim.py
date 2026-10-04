"""二次侧各站配平阀位折算（锁死规则，不提供切换）。

投运后近端站压差大、流量偏大，回水温度偏高、温差偏小；远端站反过来。配平只做
两件事：把各站的流量重新分一遍，再把目标流量折算成配平阀位。规则一旦定死就不
允许运行期切换，控制台展示、落盘审计与现场执行看到的是同一份口径：

* **配平方法＝等温差**（``EQUAL_DELTA_T``）。各站负荷 ``Q_i = G_i·ΔT_i`` 保持
  不变（阀门只搬流量、不搬热量），把全部参与站的温差拉到同一个加权平均温差
  ``ΔT* = ΣQ_i / ΣG_i``，于是目标流量 ``G_i* = Q_i / ΔT*``，参与站总流量
  守恒。不采用"等比例阻力"：那需要各站支路压差测点与管网耦合模型，现有回读
  只有供回水温度与流量，数据不支持，强行按阻力折算会把缺测站的结果带偏。
* **保远端优先**（``FAR_FIRST``）。站序按离热源由近到远给出（列表第一个是
  近端），阀位可行域受阀门可调倍率限制；目标流量落不进可行域时，按站序从最
  远端开始逐个钉死（远端先保），把残余水量顺次推给更近的站，近端站兜底。这
  是按站序定的字典序裁决，不存在"先保总流量"的第二条路；无站被钉死时总流量
  天然守恒，钉死发生时总流量短欠如实记入报告，不用远端站凑数。
* **回读缺失即排除**。回水温度、流量或当前阀位缺一项（``None`` 或非正数），
  该站标记 ``telemetry_missing``，不进配平、不参与均值，方案里它的目标阀位
  为空——不能拿缺测数据算配平。执行期阀门回读缺失属于部分失败
  （``readback_missing``）：该站按"不动"处理，其余站继续，不中断整组配平。
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass

# ---- 锁死的规则标识（审计与控制台必须原样带出，不得改值） ----------------
METHOD_EQUAL_DELTA_T = "EQUAL_DELTA_T"
PRIORITY_FAR_FIRST = "FAR_FIRST"

# ---- 配平阀物理参数（对全网同型阀固定） -----------------------------------
# 等百分比阀可调比 R：阀位行程按相对开度线性折算流通能力倍率。
TRIM_VALVE_RANGEABILITY = 50.0
# 单次配平允许的最大行程（个百分点）：再大的目标差也分多轮配平，避免一次猛关。
TRIM_MAX_TRAVEL_PCT = 30.0
# 配平阀最小开度（%）：保留支路最低流通，不允许关死。
TRIM_MIN_POSITION_PCT = 5.0
# 流量下限/上限倍率（相对当前流量）的保守硬边界，叠加在阀门可达倍率之外。
TRIM_MIN_FLOW_FACTOR = 0.05
TRIM_MAX_FLOW_FACTOR = 1.5

# 参与配平的最少站数：只剩一个站谈不上平衡。
MIN_PARTICIPANTS = 2

# 方案/结果状态。
STATUS_OK = "ok"
STATUS_PARTIAL = "partial_failure"
STATUS_INFEASIBLE = "infeasible"

# 单站处置标记。
ROW_PARTICIPANT = "participant"
ROW_TELEMETRY_MISSING = "telemetry_missing"
ROW_CLAMPED_LOW = "flow_clamped_low"
ROW_CLAMPED_HIGH = "flow_clamped_high"
ROW_READBACK_MISSING = "readback_missing"
ROW_UNCHANGED = "unchanged"


@dataclass(frozen=True)
class StationReading:
    """一个站投运后的回读：站序由它在列表里的位置（近→远）决定。"""

    station: str
    supply_c: float
    return_c: float
    flow_t_h: float
    valve_pct: float

    @classmethod
    def build(cls, raw: dict) -> "StationReading | None":
        """从回读映射构造一站；字段缺失或非正数返回 ``None`` 由调用方排除。

        供水温度允许全网统一（``common_supply_c`` 兜底），但回水温度、流量、
        当前阀位必须逐站具备，缺一即视为回读缺失。
        """

        if not isinstance(raw, dict):
            return None
        try:
            station = str(raw["station"])
            return_c = float(raw["return_c"])
            flow_t_h = float(raw["flow_t_h"])
            valve_pct = float(raw["valve_pct"])
        except (KeyError, TypeError, ValueError):
            return None
        supply_c = raw.get("supply_c", raw.get("common_supply_c"))
        try:
            supply_value = float(supply_c)  # type: ignore[arg-type]
        except (TypeError, ValueError):
            return None
        if not station or min(return_c, flow_t_h, valve_pct, supply_value) <= 0.0:
            return None
        return cls(station, supply_value, return_c, flow_t_h, max(0.0, min(100.0, valve_pct)))

    @property
    def delta_c(self) -> float:
        """本站当前供回水温差。"""

        return self.supply_c - self.return_c

    def load_t_h_c(self) -> float:
        """本站当前热负荷的 t/h·℃ 量（ΔT 折算的公共分子，1.163 在比值里约掉）。"""

        return self.flow_t_h * self.delta_c


@dataclass(frozen=True)
class TrimStationPlan:
    """一个站在配平方案里的一行（缺测站只有身份字段）。"""

    station: str
    order: int
    status: str
    supply_c: float | None
    return_c: float | None
    flow_t_h: float | None
    valve_pct: float | None
    delta_c: float | None
    target_flow_t_h: float | None
    target_valve_pct: float | None
    valve_move_pct: float | None

    def as_dict(self) -> dict:
        return {
            "station": self.station,
            "order": self.order,
            "status": self.status,
            "supply_c": _round(self.supply_c),
            "return_c": _round(self.return_c),
            "flow_t_h": _round(self.flow_t_h),
            "valve_pct": _round(self.valve_pct),
            "delta_c": _round(self.delta_c),
            "target_flow_t_h": _round(self.target_flow_t_h),
            "target_valve_pct": _round(self.target_valve_pct),
            "valve_move_pct": _round(self.valve_move_pct),
        }


@dataclass(frozen=True)
class TrimPlan:
    """一次配平的完整方案快照：控制台展示、执行器与审计共用同一份对象。"""

    method: str
    priority: str
    target_delta_c: float
    target_return_c: float | None
    total_flow_t_h: float
    total_load_kw: float
    participant_count: int
    missing_stations: tuple[str, ...]
    flow_shortfall_t_h: float
    feasible: bool
    rows: tuple[TrimStationPlan, ...]
    fingerprint: str

    def as_dict(self) -> dict:
        return {
            "method": self.method,
            "priority": self.priority,
            "target_delta_c": round(self.target_delta_c, 3),
            "target_return_c": _round(self.target_return_c),
            "total_flow_t_h": round(self.total_flow_t_h, 3),
            "total_load_kw": round(self.total_load_kw, 2),
            "participant_count": self.participant_count,
            "missing_stations": list(self.missing_stations),
            "flow_shortfall_t_h": round(self.flow_shortfall_t_h, 3),
            "feasible": self.feasible,
            "rows": [row.as_dict() for row in self.rows],
            "fingerprint": self.fingerprint,
        }

    def get(self, station: str) -> TrimStationPlan:
        """按站名取方案行。"""

        for row in self.rows:
            if row.station == station:
                return row
        raise KeyError(f"站不在配平方案里: {station}")

    def action_rows(self) -> tuple[TrimStationPlan, ...]:
        """返回需要动阀的方案行（执行序：远端在前，近端兜底）。"""

        moving = [
            row
            for row in self.rows
            if row.target_valve_pct is not None and abs(row.valve_move_pct or 0.0) > 1e-9
        ]
        return tuple(sorted(moving, key=lambda row: -row.order))


def build_trim_plan(raw_readings: list[dict]) -> TrimPlan:
    """按等温差＋保远端优先，从各站回读算出一组配平阀位。

    ``raw_readings`` 按离热源由近到远排列；不可解析的条目按回读缺失处理
    （能拿到站名就用站名，否则用占位名），缺测站不进任何均值与分摊。
    """

    readings: list[StationReading | None] = []
    seen: set[str] = set()
    for raw in raw_readings:
        reading = StationReading.build(raw)
        if reading is not None:
            if reading.station in seen:
                raise ValueError(f"站序里站名重复: {reading.station}")
            seen.add(reading.station)
        readings.append(reading)

    missing: list[str] = []
    rows: list[TrimStationPlan] = []
    for index, reading in enumerate(readings):
        if reading is None:
            label = _missing_label(raw_readings[index], index)
            missing.append(label)
            rows.append(
                TrimStationPlan(label, index, ROW_TELEMETRY_MISSING, None, None, None, None, None, None, None, None)
            )

    participants = [reading for reading in readings if reading is not None]
    total_flow = sum(item.flow_t_h for item in participants)
    total_load = sum(item.load_t_h_c() for item in participants) / 1.163

    if len(participants) < MIN_PARTICIPANTS or total_flow <= 0.0:
        for reading in participants:
            rows.append(
                TrimStationPlan(
                    reading.station,
                    _order_of(readings, reading.station),
                    ROW_TELEMETRY_MISSING,
                    reading.supply_c,
                    reading.return_c,
                    reading.flow_t_h,
                    reading.valve_pct,
                    reading.delta_c,
                    None,
                    None,
                    None,
                )
            )
        rows.sort(key=lambda row: row.order)
        return _seal(
            TrimPlan(
                METHOD_EQUAL_DELTA_T,
                PRIORITY_FAR_FIRST,
                0.0,
                None,
                total_flow,
                total_load,
                0,
                tuple(missing),
                0.0,
                False,
                tuple(rows),
                "",
            )
        )

    target_delta = sum(item.load_t_h_c() for item in participants) / total_flow
    supplies = {round(item.supply_c, 6) for item in participants}
    common_supply = next(iter(supplies)) if len(supplies) == 1 else None

    targets, statuses, shortfall = _allocate_far_first(readings, participants, target_delta)
    for reading in participants:
        target_flow = targets[reading.station]
        target_valve = _valve_position(reading, target_flow)
        rows.append(
            TrimStationPlan(
                reading.station,
                _order_of(readings, reading.station),
                statuses[reading.station],
                reading.supply_c,
                reading.return_c,
                reading.flow_t_h,
                reading.valve_pct,
                reading.delta_c,
                target_flow,
                target_valve,
                target_valve - reading.valve_pct,
            )
        )
    rows.sort(key=lambda row: row.order)
    # 可行性只看总流量短欠：远端/中途站按阀位边界钉死是保远端优先的正常结果，
    # 偏差已顺次转嫁给近端；只有近端兜底仍吞不下残余水量（短欠>0）才不可行，
    # 需再配一轮（开泵/调差）而不是硬下一个到不了的阀位。
    feasible = shortfall <= 1e-6
    return _seal(
        TrimPlan(
            METHOD_EQUAL_DELTA_T,
            PRIORITY_FAR_FIRST,
            target_delta,
            common_supply - target_delta if common_supply is not None else None,
            total_flow,
            total_load,
            len(participants),
            tuple(missing),
            shortfall,
            feasible,
            tuple(rows),
            "",
        )
    )


def _allocate_far_first(
    readings: list[StationReading | None],
    participants: list[StationReading],
    initial_delta: float,
) -> tuple[dict[str, float], dict[str, str], float]:
    """远端优先的逐站钉死分配。

    每一步把最远端待分配站的目标流量钉进该站阀位可行域：已钉死站按实际值
    占走水量，剩余站按同一个残余等温差目标 ``ΔT_剩 = Q_剩 / G_剩`` 应得
    自己那份；最后一站（近端兜底）必须吞下全部残余水量，超出可行域的部分
    截断为总流量短欠，整组方案判不可行。负荷 ``Q_i`` 视为本轮不变量。
    """

    order = {reading.station: index for index, reading in enumerate(readings) if reading is not None}
    total_flow = sum(item.flow_t_h for item in participants)
    total_load = sum(item.load_t_h_c() for item in participants)
    far_to_near = sorted(participants, key=lambda item: -order[item.station])

    targets: dict[str, float] = {}
    statuses: dict[str, str] = {}
    fixed_flow = 0.0
    fixed_load = 0.0
    shortfall = 0.0

    for index, current in enumerate(far_to_near):
        last = index == len(far_to_near) - 1
        own_load = current.load_t_h_c()
        if last:
            # 近端兜底：吞下全部残余水量（守恒总量减去已钉死站实际拿走的）。
            desired = total_flow - fixed_flow
        else:
            # 本站与尚未钉死的近端站仍处在同一个等温差目标下：
            # 剩余预算 G_预算 = 守恒总量 − 已钉死站实际水量，
            # 剩余负荷 Q_剩 = 总负荷 − 已钉死站负荷，
            # 故本站应得 G_i* = Q_i·G_预算 / Q_剩。本站被截断时，钉死值
            # 改变下一步的 G_预算，偏差自动转嫁给更近的站（远端先保）。
            desired = own_load * (total_flow - fixed_flow) / max(total_load - fixed_load, 1e-9)
        low, high = _flow_bounds(current)
        target = max(low, min(high, desired))
        targets[current.station] = target
        if target < desired - 1e-9:
            statuses[current.station] = ROW_CLAMPED_LOW
            if last:
                shortfall += desired - target
        elif target > desired + 1e-9:
            statuses[current.station] = ROW_CLAMPED_HIGH
        else:
            statuses[current.station] = ROW_PARTICIPANT
        # 钉死后按实际拿走的水量记账；负荷恒为本站固有负荷。
        fixed_flow += target
        fixed_load += own_load
    return targets, statuses, shortfall


def _flow_bounds(reading: StationReading) -> tuple[float, float]:
    """按当前开度与可调比给出本站目标流量可行域（叠加 5%~150% 硬边界）。

    等百分比阀 ``Kv = Kv满·R^(h-1)``，支路压差视为不变时 ``G ∝ √Kv``，
    因此最大可达流量倍率为 ``R^((1-h)/2)``。
    """

    travel = reading.valve_pct / 100.0
    max_factor = TRIM_VALVE_RANGEABILITY ** ((1.0 - travel) / 2.0)
    valve_low = reading.flow_t_h * TRIM_MIN_FLOW_FACTOR
    valve_high = reading.flow_t_h * min(TRIM_MAX_FLOW_FACTOR, max_factor)
    return valve_low, valve_high


def _valve_position(reading: StationReading, target_flow_t_h: float) -> float:
    """把目标流量折算成配平阀位（等百分比特性，最小开度与单轮行程限幅）。

    ``G ∝ √Kv``、``Kv = Kv满·R^(h-1)``，倍率相对当前开度：
    ``h* = h + ln((G*/G)²) / ln R``；近端压水量时 h 减小（关阀），
    远端抬水量时 h 增大（开阀）。
    """

    ratio = target_flow_t_h / reading.flow_t_h
    current_travel = reading.valve_pct / 100.0
    travel = current_travel + math.log(ratio * ratio) / math.log(TRIM_VALVE_RANGEABILITY)
    wanted = max(0.0, min(1.0, travel)) * 100.0
    wanted = max(TRIM_MIN_POSITION_PCT, min(100.0, wanted))
    move = max(-TRIM_MAX_TRAVEL_PCT, min(TRIM_MAX_TRAVEL_PCT, wanted - reading.valve_pct))
    return max(TRIM_MIN_POSITION_PCT, min(100.0, reading.valve_pct + move))


def _order_of(readings: list[StationReading | None], station: str) -> int:
    for index, reading in enumerate(readings):
        if reading is not None and reading.station == station:
            return index
    raise KeyError(station)


def _missing_label(raw: object, index: int) -> str:
    if isinstance(raw, dict):
        station = raw.get("station")
        if station is not None and str(station):
            return str(station)
    return f"missing-{index}"


def _seal(plan: TrimPlan) -> TrimPlan:
    """给方案盖指纹：执行结果与审计都凭它核对"方案有没有被人改过"。"""

    payload = plan.as_dict()
    payload["fingerprint"] = ""
    canonical = json.dumps(payload, ensure_ascii=False, sort_keys=True)
    fingerprint = hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]
    return TrimPlan(
        plan.method,
        plan.priority,
        plan.target_delta_c,
        plan.target_return_c,
        plan.total_flow_t_h,
        plan.total_load_kw,
        plan.participant_count,
        plan.missing_stations,
        plan.flow_shortfall_t_h,
        plan.feasible,
        plan.rows,
        fingerprint,
    )


def _round(value: float | None, digits: int = 2) -> float | None:
    return None if value is None else round(float(value), digits)
