"""方案、执行与审计的一致性核对。

方案、实际阀位、审计流水必须说的是同一件事：同一个 plan_id、同一组动作、
同一组目标阀位；被跳过的站不得出现任何动作或执行记录。核对结果随方案一起
上控制台——"一致"是逐条算出来的，不是写死的。
"""

from __future__ import annotations

from .execute import ExecutionReport
from .model import BalancingPlan

TARGET_TOLERANCE_PCT = 1e-6


def verify_consistency(plan: BalancingPlan, execution: ExecutionReport) -> list[str]:
    """逐条核对方案与执行，返回问题清单（空清单表示三方一致）。"""

    problems: list[str] = []
    if execution.plan_id != plan.plan_id:
        problems.append(f"执行报告 plan_id={execution.plan_id} 与方案 {plan.plan_id} 不一致")
    planned = {action.action_id: action for action in plan.actions}
    executed = {action.action_id: action for action in execution.actions}
    missing = sorted(set(planned) - set(executed))
    extra = sorted(set(executed) - set(planned))
    if missing:
        problems.append(f"方案动作未执行: {missing}")
    if extra:
        problems.append(f"执行了方案外的动作: {extra}")
    for action_id, action in planned.items():
        done = executed.get(action_id)
        if done is None:
            continue
        if abs(done.target_valve_pct - action.target_valve_pct) > TARGET_TOLERANCE_PCT:
            problems.append(
                f"{action.station} 执行目标 {done.target_valve_pct} 与方案 {action.target_valve_pct} 不符"
            )
        if done.result == "DONE" and abs(done.actual_valve_pct - action.target_valve_pct) > TARGET_TOLERANCE_PCT:
            problems.append(
                f"{action.station} 实际阀位 {done.actual_valve_pct} 未达到方案目标 {action.target_valve_pct}"
            )
    skipped = {skip.station for skip in plan.skipped}
    for action in plan.actions:
        if action.station in skipped:
            problems.append(f"跳过站 {action.station} 不应出现在方案动作里")
    for done in execution.actions:
        if done.station in skipped:
            problems.append(f"跳过站 {done.station} 不应被执行")
    return problems
