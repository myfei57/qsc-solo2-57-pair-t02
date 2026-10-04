"""配平动作的执行排序与执行器互斥。

阀位动作有先后：先把欠水的远端抬起来（开阀阶段，站序降序，远端优先），
再把贪水的近端压下去（关阀阶段，站序升序，近端先压）——先开后关，避免
近端先关死把管网压力顶高。全部动作排成全局全序，同一执行器任意时刻只被
一台阀占用；``assert_actuator_exclusion`` 把这个互斥不变量显式查一遍，
排序策略以后改动时它仍然兜底。
"""

from __future__ import annotations

from dataclasses import replace

from ..errors import InterlockError
from .model import Direction, ValveAction


def schedule_actions(actions: tuple[ValveAction, ...]) -> tuple[ValveAction, ...]:
    """把动作排成全局全序：开阀阶段在前（远端优先），关阀阶段在后（近端优先）。"""

    opens = sorted(
        (action for action in actions if action.direction is Direction.OPEN),
        key=lambda action: (-action.order, action.station),
    )
    closes = sorted(
        (action for action in actions if action.direction is Direction.CLOSE),
        key=lambda action: (action.order, action.station),
    )
    staged = [(1, action) for action in opens] + [(2, action) for action in closes]
    scheduled = tuple(
        replace(
            action,
            phase=phase,
            sequence=sequence,
            action_id=f"{action.plan_id}-A{sequence:02d}",
        )
        for sequence, (phase, action) in enumerate(staged, start=1)
    )
    assert_actuator_exclusion(scheduled)
    return scheduled


def assert_actuator_exclusion(actions: tuple[ValveAction, ...]) -> None:
    """校验执行器互斥：同一执行器不得落在同一个执行序号上（不能被几台阀同时抢）。"""

    by_actuator: dict[str, list[int]] = {}
    for action in actions:
        by_actuator.setdefault(action.actuator, []).append(action.sequence)
    for actuator, sequences in sorted(by_actuator.items()):
        if len(sequences) != len(set(sequences)):
            raise InterlockError(
                f"执行器 {actuator} 被多台阀同时抢占", component="balancing"
            )
