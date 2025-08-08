#!/usr/bin/env python3

import os
import sys

# ─── scripts/ -> 상위 폴더(lane_follower 패키지 루트) 경로를 PYTHONPATH에 추가 ───
current_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir  = os.path.abspath(os.path.join(current_dir, '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
    
import rospy
from std_msgs.msg import Float64, Int32, Bool
from sequence_manager.sequence_enum import SequenceState

class SequenceManager:
    def __init__(self):
        rospy.init_node("sequence_manager")
        rospy.loginfo("[SequenceManager] SequenceManager 초기화.")

        #--------------------초기상태, 멤버 초기화--------------------

        self.sequence = SequenceState.SLAM  # 초기값: SLAM 상태
        self.idle_speed = 0.
        self.idle_steer = 0.
        self.lane_speed = None
        self.lane_steer = None
        self.static_speed = None
        self.static_steer = None
        self.dynamic_speed = None
        self.dynamic_steer = None

        #--------------------Publisher--------------------

        self.seq_pub = rospy.Publisher("/sequence/state", Int32, queue_size=1)
        self.speed_pub = rospy.Publisher("/ctrl/speed", Float64, queue_size=1)
        self.steer_pub = rospy.Publisher("/ctrl/steering", Float64, queue_size=1)

        #--------------------Subscriber--------------------
        
        self.slam_done_sub = rospy.Subscriber("/sequence/slam_done", Bool, self.slam_done_CB)
        self.static_done_sub = rospy.Subscriber("/sequence/static_done", Bool, self.static_done_CB)
        self.dynamic_done_sub = rospy.Subscriber("/sequence/dynamic_done", Bool, self.dynamic_done_CB)

        self.lane_speed_sub = rospy.Subscriber("/lane/speed", Float64, self.lane_speed_CB)
        self.lane_steer_sub = rospy.Subscriber("/lane/steer", Float64, self.lane_steer_CB)
        self.static_speed_sub = rospy.Subscriber("/static/speed", Float64, self.static_speed_CB)
        self.static_steer_sub = rospy.Subscriber("/static/steer", Float64, self.static_steer_CB)
        self.dynamic_speed_sub = rospy.Subscriber("/dynamic/speed", Float64, self.dynamic_speed_CB)
        self.dynamic_steer_sub = rospy.Subscriber("/dynamic/steer", Float64, self.dynamic_steer_CB)

        self.rate = rospy.Rate(20)
        self.run()

    #--------------------각 미션 완료 콜백함수--------------------

    def slam_done_CB(self, msg):
        if msg.data == True:
            self.sequence = SequenceState.LANE_FOLLOWING
        elif msg.data == False:
            self.sequence = SequenceState.SLAM

    def static_done_CB(self, msg):
        if msg.data == True:
            self.sequence = SequenceState.LANE_FOLLOWING
        elif msg.data == False:
            self.sequence = SequenceState.STATIC_OBSTACLE
            
    def dynamic_done_CB(self, msg):
        if msg.data == True:
            self.sequence = SequenceState.LANE_FOLLOWING
        elif msg.data == False:
            self.sequence = SequenceState.DYNAMIC_OBSTACLE

    #--------------------각 미션별 조향각, 속도 콜백함수--------------------

    def lane_speed_CB(self, msg):
        self.lane_speed = msg.data
    
    def lane_steer_CB(self, msg):
        self.lane_steer = msg.data

    def static_speed_CB(self, msg):
        self.static_speed = msg.data
    
    def static_steer_CB(self, msg):
        self.static_steer = msg.data

    def dynamic_speed_CB(self, msg):
        self.dynamic_speed = msg.data
    
    def dynamic_steer_CB(self, msg):
        self.dynamic_steer = msg.data
            
    #--------------------시퀀스 Enum 기반으로 분기 처리하는 함수--------------------
        
    def run(self):
        while not rospy.is_shutdown():
            self.seq_pub.publish(self.sequence.value)

            # Enum 기반 분기 처리
            if self.sequence == SequenceState.SLAM:
                self.handle_slam()
            elif self.sequence == SequenceState.IDLE:
                self.handle_idle()
            elif self.sequence == SequenceState.LANE_FOLLOWING:
                self.handle_lane_following()
            elif self.sequence == SequenceState.STATIC_OBSTACLE:
                self.handle_static_obstacle()
            elif self.sequence == SequenceState.DYNAMIC_OBSTACLE:
                self.handle_dynamic_obstacle()
            else:
                rospy.logwarn_throttle(5.0, f"[SequenceManager] 알 수 없는 시퀀스: {self.sequence}")

            self.rate.sleep()

    #--------------------각 시퀀스 별 처리 함수--------------------

    def handle_slam(self):
        rospy.loginfo_throttle(5.0, "[SequenceManager] SLAM 진행 중...")

    def handle_idle(self):
        rospy.loginfo_throttle(5.0, "[SequenceManager] 대기 상태입니다.")

    def handle_lane_following(self):
        rospy.loginfo_throttle(2.0, "[SequenceManager] 차선 추종 중...")
        self.speed_pub.publish(self.lane_speed)
        self.steer_pub.publish(self.lane_steer)

    def handle_static_obstacle(self):
        rospy.loginfo_throttle(2.0, "[SequenceManager] 정적 장애물 회피 중...")
        self.speed_pub.publish(self.static_speed)
        self.steer_pub.publish(self.static_steer)

    def handle_dynamic_obstacle(self):
        rospy.loginfo_throttle(2.0, "[SequenceManager] 동적 장애물 회피 중...")
        self.speed_pub.publish(self.dynamic_speed)
        self.steer_pub.publish(self.dynamic_steer)


if __name__ == "__main__":
    try:
        SequenceManager()
    except rospy.ROSInterruptException:
        pass
