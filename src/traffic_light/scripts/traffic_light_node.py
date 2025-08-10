#!/usr/bin/env python3

import os
import sys

# ─── scripts/ -> 상위 폴더(lane_follower 패키지 루트) 경로를 PYTHONPATH에 추가 ───
current_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir  = os.path.abspath(os.path.join(current_dir, '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
    
import rospy
from std_msgs.msg import String, Bool
from morai_msgs.msg import GetTrafficLightStatus  # MORAI 원본 메세지 사용
from traffic_light.perception import TrafficLightPerception
from traffic_light.decision import decide

class TrafficLightNode:
    def __init__(self):
        # params
        topic_in = rospy.get_param("~traffic_topic", "/GetTrafficLightStatus")
        topic_sem = rospy.get_param("~semantic_out", "/traffic/semantic")
        topic_stop = rospy.get_param("~stop_out", "/traffic/stop")
        topic_cau = rospy.get_param("~caution_out", "/traffic/caution")
        topic_lft = rospy.get_param("~left_go_out", "/traffic/left_go")
        topic_str = rospy.get_param("~straight_go_out", "/traffic/straight_go")

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
        semantic = self.percep.update(msg.trafficLightStatus)         # 'RED'|'YELLOW'|'STRAIGHT'|'LEFT'
        rospy.loginfo_throttle(0.5, f"[TL] semantic={semantic} raw={self.percep.last_raw}")
        act = decide(semantic)                     # TLAction

        # publish
        self.pub_sem.publish(String(data=act.semantic))
        self.pub_stop.publish(Bool(data=act.stop))
        self.pub_cau.publish(Bool(data=act.caution))
        self.pub_lft.publish(Bool(data=act.go_left))
        self.pub_str.publish(Bool(data=act.go_straight))

        rospy.loginfo_throttle(1.0, f"[TL] semantic={semantic}")

    def spin(self):
        rospy.loginfo("traffic_light node started.")
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("traffic_light_mission")
    TrafficLightNode().spin()
