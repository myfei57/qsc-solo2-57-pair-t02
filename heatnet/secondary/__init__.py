"""二次网。

二次网负责把一次网送来的热量分到末端：循环泵建立流量，补水定压把系统压力顶在
许可带内，回水温度按水力平衡收敛。一个循环无论走成功还是半途失败，泵都必须停稳。

投运后的各站二次侧温差失衡由配平服务处理：等温差＋保远端优先的阀位方案在
:mod:`heatnet.secondary.trim` 锁死，串行执行与部分失败语义在
:mod:`heatnet.secondary.actuator`，展示、执行与审计的串联在
:mod:`heatnet.secondary.trim_service`。
"""

from __future__ import annotations

from .actuator import (
    SimulatedTrimDriver,
    TrimActuatorBus,
    TrimDriver,
    TrimRun,
    TrimStationResult,
    execute_trim,
)
from .balance import BalanceReport, hydraulic_balance
from .cycle import run_cycle
from .loop import SecondaryLoop
from .makeup import makeup_then_pressurise
from .state import SecondarySnapshot
from .trim import METHOD_EQUAL_DELTA_T, PRIORITY_FAR_FIRST, StationReading, TrimPlan, TrimStationPlan, build_trim_plan
from .trim_service import TrimService, station_deviations

__all__ = [
    "METHOD_EQUAL_DELTA_T",
    "PRIORITY_FAR_FIRST",
    "BalanceReport",
    "SecondaryLoop",
    "SecondarySnapshot",
    "SimulatedTrimDriver",
    "StationReading",
    "TrimActuatorBus",
    "TrimDriver",
    "TrimPlan",
    "TrimRun",
    "TrimService",
    "TrimStationPlan",
    "TrimStationResult",
    "build_trim_plan",
    "execute_trim",
    "hydraulic_balance",
    "makeup_then_pressurise",
    "run_cycle",
    "station_deviations",
]
