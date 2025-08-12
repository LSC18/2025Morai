#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys

# ─── scripts/ -> 상위 폴더(lane_follower 패키지 루트) 경로를 PYTHONPATH에 추가 ───
current_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir  = os.path.abspath(os.path.join(current_dir, '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
    
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64, Bool
from cv_bridge import CvBridge, CvBridgeError

from lane_follower.perception import Perception
from lane_follower.detection import Detection

class LaneFollowerNode:
    """
    ROS node that subscribes to a compressed camera image topic,
    runs lane perception + detection, computes steering & speed,
    and publishes control commands.
    """
    def __init__(self):
        # --- Node & parameters ---
        rospy.init_node('lane_follower', anonymous=False)
        # topics (can be overriden via rosparam)
        image_topic    = rospy.get_param('~image_topic',    '/image_jpeg/compressed')
        steer_topic    = rospy.get_param('~steering_topic', '/lane/steer')

        # sliding‐window parameters
        nwindows  = rospy.get_param('~nwindows',  12)
        margin    = rospy.get_param('~margin',    105)
        minpix    = rospy.get_param('~minpix',     5)
        threshold = rospy.get_param('~threshold',100)
        # control “midrange” half‐width
        self.midrange = rospy.get_param('~midrange', 260) # 310이 베스트인듯? 260은 좌회전, 우회전때
        
        # --- direction mode ---
        self.mode = "normal"
        
        # --- flag ---
        self.stopline_flag = False
        
        # --- Helpers ---
        self.bridge = CvBridge()
        self.perc   = Perception()
        self.det    = Detection(nwindows, margin, minpix, threshold)
        
        # --- Publishers & Subscriber ---
        self.steer_pub = rospy.Publisher(steer_topic, Float64, queue_size=1)
        self.stopline_pub = rospy.Publisher("/lane/stopline", Bool, queue_size=1)
        
        rospy.Subscriber(image_topic, CompressedImage, self._image_cb, queue_size=1)
        rospy.Subscriber("/lane/mode", Float64, self._mode_cb, queue_size=1)

        rospy.loginfo("lane_follower node started; listening to %s", image_topic)

    def _mode_cb(self, msg:Float64):
        if msg.data < 0:
            self.mode = "left_only"
        elif msg.data > 0:
            self.mode = "right_only"
        else:
            self.mode = "normal"

    def _image_cb(self, msg: CompressedImage):
        # 1) Convert ROS msg to OpenCV image
        try:
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr("CvBridge conversion failed: %s", e)
            return

        # 2) Perception pipeline
        self.perc.img_init(cv_img)    # set img, img_hsv, x/y
        self.perc.img_transform()     # color threshold → binary mask
        self.perc.img_warp()          # perspective warp → top-down view

        # 3) Detection: sliding window → left/right lane lines
        l_lane, r_lane, img = self.det.sliding_window(self.perc.img, mode=self.mode)
        
        # ?) 정지선 검출 및 발행
        self.stopline_flag = self.det.detect_stopline(
            self.perc.img,
            band_from_bottom=0,
            band_height=80,
            row_ratio_thr=0.4,
            hold_frames=3
        )
        self.stopline_pub.publish(Bool(self.stopline_flag))

        # 4) Control computation
        ctrl = self.det.compute_control(
            x             = self.perc.x,
            l_lane        = l_lane,
            r_lane        = r_lane,
            midrange      = self.midrange,
            mode          = self.mode,
            lane_width_px = 286,
            delta_px      = 10
        )
        
        steer = self.det.shape_steer(ctrl, p=1.6, scale=3.5)
        
        # 5) Publish control commands
        rospy.loginfo(f"steering : {Float64(steer)}\n")
        self.steer_pub.publish(Float64(steer))
        
        # 6) 디버그 시각화 : BEV 이진 이미지
        cv2.imshow("BEV Binary", img)
        cv2.waitKey(1)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    node = LaneFollowerNode()
    node.spin()
