#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool
from morai_msgs.msg import GetTrafficLightStatus  # MORAI 원본 메세지 사용
from traffic_light.perception import TrafficLightPerception
from traffic_light.decision import decide

class TrafficLightNode:
    def __init__(self):
        # params
        topic_in = rospy.get_param("~traffic_topic", "/GetTrafficStatus")
        topic_sem = rospy.get_param("~semantic_out", "/tl/semantic")
        topic_stop = rospy.get_param("~stop_out", "/tl/stop")
        topic_cau = rospy.get_param("~caution_out", "/tl/caution")
        topic_lft = rospy.get_param("~left_go_out", "/tl/left_go")
        topic_str = rospy.get_param("~straight_go_out", "/tl/straight_go")

        # pubs
        self.pub_sem = rospy.Publisher(topic_sem, String, queue_size=1)
        self.pub_stop = rospy.Publisher(topic_stop, Bool, queue_size=1)
        self.pub_cau = rospy.Publisher(topic_cau, Bool, queue_size=1)
        self.pub_lft = rospy.Publisher(topic_lft, Bool, queue_size=1)
        self.pub_str = rospy.Publisher(topic_str, Bool, queue_size=1)

        # perception
        self.percep = TrafficLightPerception()

        # sub
        rospy.Subscriber(topic_in, GetTrafficLightStatus, self._cb, queue_size=10)

    def _cb(self, msg: GetTrafficLightStatus):
        semantic = self.percep.update(msg)         # 'RED'|'YELLOW'|'STRAIGHT'|'LEFT'
        rospy.loginfo_throttle(0.5, f"[TL] semantic={semantic} raw={self.percep.last_raw}")
        act = decide(semantic)                     # TLAction

        # publish
        self.pub_sem.publish(String(data=act.semantic))
        self.pub_stop.publish(Bool(data=act.stop))
        self.pub_cau.publish(Bool(data=act.caution))
        self.pub_lft.publish(Bool(data=act.go_left))
        self.pub_str.publish(Bool(data=act.go_straight))

        rospy.loginfo_throttle(1.0, f"[TL] raw={self.percep.last_raw} semantic={semantic} "
                                    f"stop={act.stop} caution={act.caution} "
                                    f"L={act.go_left} S={act.go_straight}")

    def spin(self):
        rospy.loginfo("traffic_light node started.")
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("traffic_light_mission")
    TrafficLightNode().spin()
