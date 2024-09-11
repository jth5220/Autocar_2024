#!/usr/bin/env python3

import numpy as np
import time

import rospy
from ackermann_msgs.msg import AckermannDrive

class ForERPTest():
    def __init__(self):
        self.first_time = None
        # ROS
        rospy.init_node('for_dr_test', anonymous=True)
        self.cmd_pub = rospy.Publisher("/erp_command", AckermannDrive, queue_size=10)
        
        self.timer_cmd = rospy.Timer(rospy.Duration(0.1), self.timer_callback_cmd)
        return
    
    def timer_callback_cmd(self, event):
        vel = 1.5 # m/s
        steer = self.make_steer_value() # degrees
        
        # ROS Publish
        cmd_msg = AckermannDrive()
        cmd_msg.steering_angle = steer # 최종 입력 조향각
        cmd_msg.speed = vel # 최종 입력 속도
        cmd_msg.jerk = 0 # 최종 입력 브레이크
        self.cmd_pub.publish(cmd_msg)
        
        print("발행 속도:", vel)
        print("발행 조향각:", steer)
        print()
        return
    
    def make_steer_value(self):
        # # 1. -10도로 계속 주기
        # steer = -10
        
        # 2. 0s~5s: 조향 0도
        #    5s~  : 조향 10도
        if self.first_time is None:
            self.first_time = time.time()
            return 0.
        
        if time.time() - self.first_time < 10:
            steer = 0.
            
        else:
            steer = -10.
        
        return steer
    
if __name__ == '__main__':
    node = ForERPTest()
    rospy.spin()