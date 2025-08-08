# traffic_light/decision.py
"""Traffic light decision logic.

State mapping:

State       is_stop   is_left
---------------------------------
RED         True      False
YELLOW      True      False
LEFT        False     True
STRAIGHT    False     False
(other)     True      False
"""

from typing import Tuple

class TrafficLightDecision:
    """Convert semantic traffic light state to stop/left flags."""

    def get_command(self, state: str) -> Tuple[bool, bool]:
        # 기본적으로 문자열을 대문자로 맞춰줍니다.
        if not isinstance(state, str):
            return True, False

        key = state.upper()

        if key == 'RED':
            return True, False
        if key == 'YELLOW':
            return True, False
        if key == 'LEFT':
            return False, True
        if key == 'STRAIGHT':
            return False, False

        # 알 수 없는 상태는 안전하게 정지
        return True, False
