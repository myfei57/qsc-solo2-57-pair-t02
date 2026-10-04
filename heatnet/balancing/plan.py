"""等温差法配平计算。

方法定死为等温差法：目标温差取流量加权平均 ΔT* = Σ(Gᵢ·ΔTᵢ)/ΣGᵢ，各站目标
流量之和因此恒等于当前总流量——总流量守恒是方法自带的硬约束，"先保远端还是
先保总流量"不构成取舍：总流量由方法保住，站间先后只由站序决定（见
:mod:`~heatnet.balancing.sequence`）。不采用等比例阻力法：它要求逐站实测阀门
阻力特性，投运期拿不到可靠数据，而每站的供回水温差都有回读。

近端贪水的站流量偏大、温差偏小，按 ΔT* 折算要关小；远端欠水的站温差偏大，要
开大。阀位按相对流量偏差经线性化增益折算，单周期步幅限幅，防止一把拧过头
引起管网振荡。
"""

from __future__ import annotations

from ..config import BalancingLimits
from ..errors import InterlockError
from .model import (
    FAILURE_STATUSES,
    BalancingPlan,
    Direction,
    SkippedStation,
    StationReading,
    StationStatus,
    ValveAction,
)
from .sequence import schedule_actions

METHOD = "EQUAL_DELTA_T"  # 配平方法：等温差法（定死，不提供其他方法选项）


def compute_plan(
    readings: list[StationReading],
    limits: BalancingLimits,
    plan_id: str,
    created_ts: float,
) -> BalancingPlan:
    """按各站回读计算一轮配平方案。

    回读缺失或非法的站不参与目标温差计算、不下发动作（按不动处理），其余站
    继续——这是配平的部分失败口径，缺失回读的站绝不能进配平公式。
    """

    skipped: list[SkippedStation] = []
    valid: list[tuple[StationReading, float]] = []
    for reading in readings:
        if reading.readback_missing():
            skipped.append(
                SkippedStation(
                    station=reading.station,
                    order=reading.order,
                    status=StationStatus.SKIPPED_NO_READBACK,
                    reason="回读缺失（供温/回温/流量），该站按不动处理",
                    valve_pct=reading.valve_pct,
                    actuator=reading.actuator_id(),
                    supply_c=reading.supply_c,
                    return_c=reading.return_c,
                    flow_t_h=reading.flow_t_h,
                )
            )
            continue
        delta = reading.delta_c()
        flow = reading.flow_t_h
        if delta is None or delta <= 0 or flow is None or flow <= 0:
            skipped.append(
                SkippedStation(
                    station=reading.station,
                    order=reading.order,
                    status=StationStatus.SKIPPED_INVALID,
                    reason=f"回读非法（温差 {delta}℃ / 流量 {flow}t/h），该站按不动处理",
                    valve_pct=reading.valve_pct,
                    actuator=reading.actuator_id(),
                    supply_c=reading.supply_c,
                    return_c=reading.return_c,
                    flow_t_h=reading.flow_t_h,
                    delta_c=delta,
                )
            )
            continue
        valid.append((reading, delta))

    total_before = sum(reading.flow_t_h for reading, _ in valid if reading.flow_t_h)
    if not valid:
        return BalancingPlan(
            plan_id=plan_id,
            created_ts=created_ts,
            method=METHOD,
            target_delta_c=0.0,
            total_flow_before_t_h=0.0,
            total_flow_after_t_h=0.0,
            status="ALL_SKIPPED",
            actions=(),
            skipped=tuple(skipped),
        )

    target_delta = sum(reading.flow_t_h * delta for reading, delta in valid) / total_before

    actions: list[ValveAction] = []
    for reading, delta in valid:
        deviation = delta - target_delta
        target_flow = reading.flow_t_h * delta / target_delta
        if abs(deviation) <= limits.delta_t_deadband_c:
            skipped.append(
                SkippedStation(
                    station=reading.station,
                    order=reading.order,
                    status=StationStatus.HOLD_DEADBAND,
                    reason=f"温差偏差 {deviation:+.2f}℃ 在死区 ±{limits.delta_t_deadband_c}℃ 内",
                    valve_pct=reading.valve_pct,
                    actuator=reading.actuator_id(),
                    supply_c=reading.supply_c,
                    return_c=reading.return_c,
                    flow_t_h=reading.flow_t_h,
                    delta_c=delta,
                    deviation_c=deviation,
                )
            )
            continue
        relative = delta / target_delta - 1.0
        step = max(-limits.max_step_pct, min(limits.max_step_pct, limits.valve_gain * relative))
        raw_target = reading.valve_pct + step
        clamped = max(limits.valve_min_pct, min(limits.valve_max_pct, raw_target))
        if (clamped - reading.valve_pct) * step <= 0:
            # 限幅把动作顶反或顶没：阀已在限位，朝需要的方向动不了。
            skipped.append(
                SkippedStation(
                    station=reading.station,
                    order=reading.order,
                    status=StationStatus.AT_LIMIT,
                    reason="阀位已在限位，朝需要的方向动不了",
                    valve_pct=reading.valve_pct,
                    actuator=reading.actuator_id(),
                    supply_c=reading.supply_c,
                    return_c=reading.return_c,
                    flow_t_h=reading.flow_t_h,
                    delta_c=delta,
                    deviation_c=deviation,
                )
            )
            continue
        status = (
            StationStatus.ACTION
            if abs(raw_target - clamped) < 1e-9
            else StationStatus.CLAMPED
        )
        direction = Direction.OPEN if clamped > reading.valve_pct else Direction.CLOSE
        actions.append(
            ValveAction(
                plan_id=plan_id,
                station=reading.station,
                order=reading.order,
                actuator=reading.actuator_id(),
                direction=direction,
                supply_c=reading.supply_c,
                return_c=reading.return_c,
                delta_c=delta,
                deviation_c=deviation,
                flow_t_h=reading.flow_t_h,
                target_flow_t_h=target_flow,
                valve_pct=reading.valve_pct,
                target_valve_pct=clamped,
                status=status,
            )
        )

    # 总流量守恒是硬约束：全体有效站按等温差折算的目标流量之和必须等于当前
    # 总流量。数学上恒成立，这里显式校验，防止折算公式被改动后悄悄破坏守恒。
    ideal_after = sum(reading.flow_t_h * delta / target_delta for reading, delta in valid)
    if abs(ideal_after - total_before) > limits.total_flow_rel_tol * total_before:
        raise InterlockError("等温差折算破坏总流量守恒，方案作废", component="balancing")

    scheduled = schedule_actions(tuple(actions))
    held_flow = sum(
        skip.flow_t_h
        for skip in skipped
        if skip.status in (StationStatus.HOLD_DEADBAND, StationStatus.AT_LIMIT)
        and skip.flow_t_h is not None
    )
    total_after = sum(action.target_flow_t_h for action in scheduled) + held_flow
    status = _plan_status(scheduled, skipped)
    return BalancingPlan(
        plan_id=plan_id,
        created_ts=created_ts,
        method=METHOD,
        target_delta_c=target_delta,
        total_flow_before_t_h=total_before,
        total_flow_after_t_h=total_after,
        status=status,
        actions=scheduled,
        skipped=tuple(skipped),
    )


def _plan_status(actions: tuple[ValveAction, ...], skipped: list[SkippedStation]) -> str:
    """汇总方案状态：有失败跳过即部分失败，无动作即本轮不动。"""

    if any(skip.status in FAILURE_STATUSES for skip in skipped):
        return "PARTIAL"
    if not actions:
        return "NO_ACTION"
    return "OK"
