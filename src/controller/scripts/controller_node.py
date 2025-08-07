#!/usr/bin/env python3
# -*- coding: utf-8 -*-
    
import rospy
from std_msgs.msg import Float64

class ControllerNode:
    def __init__(self):
        rospy.init_node('controller', anonymous=False)
        
        self.steer_pub = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        rospy.Subscriber("/ctrl/steering", Float64, self.steering_CB, queue_size=1)
        rospy.Subscriber("/ctrl/speed", Float64, self.speed_CB, queue_size=1)
        
        rospy.loginfo("controller node started.")

    def steering_CB(self, msg):
        rospy.loginfo(f"steer : {msg}")
        self.steer_pub.publish(msg)
        
    def speed_CB(self, msg):
        rospy.loginfo(f"speed : {msg}")
        self.speed_pub.publish(msg)
        
    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    node = ControllerNode()
    node.spin()
