# traffic_light/scripts/traffic_light_node.py
import rospy
from std_msgs.msg import Int32, Bool
from traffic_light.perception import TrafficLightPerception
from traffic_light.decision import TrafficLightDecision

class TrafficLightNode:
    """
    ROS node for the traffic light mission.

    state → /traffic/is_stop, /traffic/is_left
      RED       →  True, False
      YELLOW    →  True, False
      LEFT      →  False, True
      STRAIGHT  →  False, False
      other     →  True, False
    """

    def __init__(self) -> None:
        self.perception = TrafficLightPerception()
        self.decision = TrafficLightDecision()

        # 이 노드는 두 개의 Bool 토픽을 퍼블리시합니다.
        self.stop_pub = rospy.Publisher('/traffic/is_stop', Bool, queue_size=1)
        self.left_pub = rospy.Publisher('/traffic/is_left', Bool, queue_size=1)

        # 신호등 입력 토픽 (기본: /traffic_light)
        tl_topic = rospy.get_param('~traffic_light_topic', '/traffic_light')
        rospy.Subscriber(tl_topic, Int32, self._light_cb, queue_size=1)

        rospy.loginfo(f"[TrafficLightNode] Listening for traffic light on {tl_topic}")

    def _light_cb(self, msg: Int32) -> None:
        # raw 메시지를 의미 있는 상태로 변환
        state = self.perception.update(msg)

        # 상태를 is_stop / is_left 플래그로 변환
        is_stop, is_left = self.decision.get_command(state)

        # 플래그 퍼블리시
        self.stop_pub.publish(Bool(is_stop))
        self.left_pub.publish(Bool(is_left))

        rospy.logdebug(
            f"[TrafficLightNode] state={state} -> is_stop={is_stop}, is_left={is_left}"
        )

    def spin(self) -> None:
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('traffic_light_mission')
    node = TrafficLightNode()
    node.spin()
