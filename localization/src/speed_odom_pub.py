#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

class SpeedOdomPub():
    def __init__(self):
        self._set_ros()
        return
    
    def _set_ros(self):
        # ROS
        rospy.init_node('speed_odom')
        self.speed_sub = rospy.Subscriber('/encoder/data', Float32, self.callback_speed)
        self.speed_pub = rospy.Publisher('/speed/odom', Odometry, queue_size=10)
        return
    
    def callback_speed(self, speed_msg):
        speed_odom_msg = Odometry()
        speed_odom_msg.header.frame_id = 'base_link'
        speed_odom_msg.twist.twist.linear.x = speed_msg.data

        self.speed_pub.publish(speed_odom_msg)
        return
    

if __name__ == "__main__":
    try:
        # ROS
        sop = SpeedOdomPub()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass