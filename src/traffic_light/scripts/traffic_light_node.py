import rospy
from std_msgs.msg import Int32, Float64
from traffic_light.perception import TrafficLightPerception
from traffic_light.decision import TrafficLightDecision

class TrafficLightNode:
    def __init__(self):
        self.perception = TrafficLightPerception()
        self.decision = TrafficLightDecision()
        # 파라미터로 상태별 목표 속도 오버라이드
        ...
        self.steer_pub = rospy.Publisher('/ctrl/steering', Float64, queue_size=1)
        self.speed_pub = rospy.Publisher('/ctrl/speed', Float64, queue_size=1)
        tl_topic = rospy.get_param('~traffic_light_topic', '/traffic_light')
        rospy.Subscriber(tl_topic, Int32, self._light_cb)

    def _light_cb(self, msg):
        state = self.perception.update(msg)         # 메시지에서 상태 추출
        steering, speed = self.decision.get_command(state)
        self.steer_pub.publish(Float64(steering))   # 조향각 퍼블리시
        self.speed_pub.publish(Float64(speed))      # 속도 퍼블리시

if __name__ == '__main__':
    rospy.init_node('traffic_light_mission')
    TrafficLightNode().spin()