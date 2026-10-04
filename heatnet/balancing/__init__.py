"""二次网配平。

投运后各站二次侧温差参差不齐：近端贪水、远端欠水，值班员只能挨个站手动拧阀。
本包按各站回水温度与流量算出一组配平阀位（等温差法，方法定死），把近端压一压、
远端抬一抬，给出配平方案与每站偏差；动作先后按站序排定、执行器互斥；回读缺失
的站按不动处理（部分失败），其余站继续。方案供控制台展示并落审计，方案、实际
阀位与审计三方一致。
"""

from .execute import ExecutedAction, ExecutionReport, execute_plan
from .model import (
    FAILURE_STATUSES,
    BalancingPlan,
    Direction,
    SkippedStation,
    StationReading,
    StationStatus,
    ValveAction,
)
from .plan import METHOD, compute_plan
from .sequence import assert_actuator_exclusion, schedule_actions
from .verify import verify_consistency

__all__ = [
    "FAILURE_STATUSES",
    "METHOD",
    "BalancingPlan",
    "Direction",
    "ExecutedAction",
    "ExecutionReport",
    "SkippedStation",
    "StationReading",
    "StationStatus",
    "ValveAction",
    "assert_actuator_exclusion",
    "compute_plan",
    "execute_plan",
    "schedule_actions",
    "verify_consistency",
]
