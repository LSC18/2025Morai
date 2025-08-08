from enum import IntEnum

class SequenceState(IntEnum):
    SLAM = -1
    IDLE = 0
    LANE_FOLLOWING = 1
    STATIC_OBSTACLE = 2
    DYNAMIC_OBSTACLE = 3
    TRAFFIC_LIGHT = 4
    TURN_LEFT = 5
    TURN_RIGHT = 6
    # 필요 시 이후 상태 계속 추가 가능
