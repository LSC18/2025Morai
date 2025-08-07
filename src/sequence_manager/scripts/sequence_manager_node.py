#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
from sequence_enum import SequenceState

class SequenceManager:
    def __init__(self):
        rospy.init_node("sequence_manager")

        self.sequence = SequenceState.SLAM  # 초기값: SLAM 대기 상태
        self.seq_pub = rospy.Publisher("/sequence/state", Int32, queue_size=1)

        # SLAM 완료 체크 (파라미터 기반)
        self.slam_check_timer = rospy.Timer(rospy.Duration(1.0), self.check_slam_done)

        self.rate = rospy.Rate(10)
        self.run()

    def check_slam_done(self, event):
        if rospy.has_param("/slam_done") and rospy.get_param("/slam_done") is True:
            if self.sequence == SequenceState.SLAM:
                rospy.loginfo("[SequenceManager] SLAM 완료 감지됨. 차선 추종으로 이동.")
                self.sequence = SequenceState.LANE_FOLLOWING

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

    def handle_slam(self):
        rospy.loginfo_throttle(5.0, "[SequenceManager] SLAM 진행 중...")

    def handle_idle(self):
        rospy.loginfo_throttle(5.0, "[SequenceManager] 대기 상태입니다.")

    def handle_lane_following(self):
        rospy.loginfo_throttle(2.0, "[SequenceManager] 차선 추종 중...")
        # 예: 특정 조건 만족 시 다음 시퀀스로
        # self.sequence = SequenceState.STATIC_OBSTACLE

    def handle_static_obstacle(self):
        rospy.loginfo_throttle(2.0, "[SequenceManager] 정적 장애물 회피 중...")

    def handle_dynamic_obstacle(self):
        rospy.loginfo_throttle(2.0, "[SequenceManager] 동적 장애물 회피 중...")

if __name__ == "__main__":
    try:
        SequenceManager()
    except rospy.ROSInterruptException:
        pass
