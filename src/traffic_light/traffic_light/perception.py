from typing import Any, Optional

class TrafficLightPerception:
    """Decode a raw traffic light ``Int32`` into ``'RED'``, ``'YELLOW'``, ``'STRAIGHT'`` or ``'LEFT'``."""

    def __init__(self) -> None:
        self.last_raw: Optional[int] = None

    def _decode_bitfield(self, raw: int) -> str:
        """Decode a bitfield-coded value (3 bits per phase) into a state.

        Each three-bit group encodes 0=red, 1=yellow or 2=green.  Green has
        precedence over yellow, and yellow over red.
        """
        left_phase = raw & 0b111
        straight_phase = (raw >> 3) & 0b111

        left_phase = min(left_phase, 2)
        straight_phase = min(straight_phase, 2)

        if straight_phase == 2:
            return 'STRAIGHT'
        if left_phase == 2:
            return 'LEFT'
        if straight_phase == 1 or left_phase == 1:
            return 'YELLOW'
        return 'RED'

    def _decode_raw(self, raw: int) -> str:
        if raw == 1:
            return 'RED'
        if raw == 4:
            return 'YELLOW'
        if raw == 16:
            return 'STRAIGHT'
        if raw == 33:
            return 'LEFT'
        return 'RED'

    def update(self, msg: Any) -> str:
        """Decode an incoming message to a state; bitfield for values >3 else simple mapping."""
        # raw_value = getattr(msg, '.trafficLightStatus', None)
        raw_value = int(msg)
        self.last_raw = raw_value

        if raw_value is None:
            return 'RED'

        try:
            if raw_value > 3:
                return self._decode_raw(int(raw_value))
            return self._decode_raw(int(raw_value))
        except Exception:
            return 'RED'
