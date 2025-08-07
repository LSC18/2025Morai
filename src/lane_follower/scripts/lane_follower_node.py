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
from std_msgs.msg import Float64
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
        steer_topic    = rospy.get_param('~steering_topic', '/ctrl/steering')
        speed_topic    = rospy.get_param('~speed_topic',    '/ctrl/speed')

        # sliding‐window parameters
        nwindows  = rospy.get_param('~nwindows',  12)
        margin    = rospy.get_param('~margin',    60)
        minpix    = rospy.get_param('~minpix',     5)
        threshold = rospy.get_param('~threshold',100)
        # control “midrange” half‐width
        self.midrange = rospy.get_param('~midrange', 300)

        # --- Helpers ---
        self.bridge = CvBridge()
        self.perc   = Perception()
        self.det    = Detection(nwindows, margin, minpix, threshold)

        # --- Publishers & Subscriber ---
        self.steer_pub = rospy.Publisher(steer_topic, Float64, queue_size=1)
        self.speed_pub = rospy.Publisher(speed_topic, Float64, queue_size=1)
        rospy.Subscriber(image_topic, CompressedImage, self._image_cb, queue_size=1)

        rospy.loginfo("lane_follower node started; listening to %s", image_topic)

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
        l_lane, r_lane = self.det.sliding_window(self.perc.img)

        # 4) Control computation
        ctrl = self.det.compute_control(
            x        = self.perc.x,
            l_lane   = l_lane,
            r_lane   = r_lane,
            midrange = self.midrange
        )
        speed = self.perc.speed  # default or modified in perception

        # 5) Publish control commands
        rospy.loginfo(f"speed : {Float64(speed)}\nsteering : {Float64(ctrl)}\n")
        self.steer_pub.publish(Float64(ctrl))
        self.speed_pub.publish(Float64(speed))

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    node = LaneFollowerNode()
    node.spin()
