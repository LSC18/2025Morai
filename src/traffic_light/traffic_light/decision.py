#!/usr/bin/env python3
from dataclasses import dataclass

@dataclass(frozen=True)
class TLAction:
    semantic: str       # 'RED'|'YELLOW'|'STRAIGHT'|'LEFT'
    stop: bool          # 정지해야 하는가 (RED만 True)
    caution: bool       # 주의(감속 등) 신호 (YELLOW만 True)
    go_left: bool       # 좌회전 진행 플래그 (LEFT)
    go_straight: bool   # 직진 진행 플래그 (STRAIGHT)

def decide(semantic: str) -> TLAction:
    if semantic == "LEFT":
        return TLAction(semantic, False, False, True, False)
    if semantic == "STRAIGHT":
        return TLAction(semantic, False, False, False, True)
    if semantic == "YELLOW":
        return TLAction(semantic, False, True,  False, False)
    # RED 또는 알 수 없는 값은 정지
    return TLAction("RED", True, False, False, False)
