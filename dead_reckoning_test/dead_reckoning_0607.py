#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import TransformStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf

WHEELBASE = 1.04
L_front = 0.7
L_rear = WHEELBASE - L_front

TRACK = 0.985

def normalize_angle(angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
            
        return angle
    
class DeadReckoning():
    def __init__(self):
        # utm -> odom tf 변환
        self.target_position = None
        self.target_orientation = None
        self.ready_tf_utm_to_odom = True
        self.init_y = None
        self.br = tf.TransformBroadcaster()
        
        # odometry 관련 변수
        self.x = 0
        self.y = 0
        self.imu_x = 0
        self.imu_y = 0
        self.psi = 0
        self.imu_psi = 0
        self.vel_right = 0
        self.steer = 0
        
        # time
        self.prev_time = None
        
        # ROS
        rospy.init_node('dead_reckoning')
        self.speed_sub = rospy.Subscriber('/cur_speed', Float32, self.callback_vel)
        self.epr_cmd_sub = rospy.Subscriber("/erp_command", AckermannDrive, self.callback_cmd)
        self.location_sub = rospy.Subscriber('/location_corrected', Odometry, self.callback_location)
        
        self.dead_reckoning_pub = rospy.Publisher('/dead_reckoning', Odometry, queue_size=10)
        self.dead_reckoning_imu_pub = rospy.Publisher('/dead_reckoning_imu',Odometry, queue_size=10)
        self.timer_deadreckoning = rospy.Timer(rospy.Duration(0.1), self.timer_callback_dr)
        return
    
    def callback_location(self, location_msg):
        if self.ready_tf_utm_to_odom is True:
            self.target_position = location_msg.pose.pose.position
            self.target_orientation = location_msg.pose.pose.orientation

            self.ready_tf_utm_to_odom = False
        
        # utm -> odom
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "utm"
        t.child_frame_id = "odom"

        t.transform.translation.x = self.target_position.x
        t.transform.translation.y = self.target_position.y
        t.transform.translation.z = self.target_position.z
        t.transform.rotation = self.target_orientation

        # Send the transformation
        self.br.sendTransformMessage(t)
        
        car_yaw = euler_from_quaternion([location_msg.pose.pose.orientation.x, location_msg.pose.pose.orientation.y,\
                                         location_msg.pose.pose.orientation.z, location_msg.pose.pose.orientation.w])[2] 
        int_yaw = euler_from_quaternion([self.target_orientation.x,self.target_orientation.y,self.target_orientation.z,self.target_orientation.w])[2]
        
        # imu에서 받아온 방향값
        self.imu_psi = normalize_angle(car_yaw - int_yaw)
        
        return
    
    def callback_vel(self, speed_msg):
        self.vel_right = speed_msg.data
    
    def callback_cmd(self, cmd_msg):
        self.steer = -np.radians(cmd_msg.steering_angle)
        return
    
    def timer_callback_dr(self, event):
        self.dead_reckoning()
        
    def dead_reckoning(self):
        if self.prev_time is None:
            self.prev_time = time.time()
            return
            
        # dt 계산
        cur_time = time.time()
        dt = cur_time - self.prev_time
        self.prev_time = cur_time
        
        beta = np.arctan2(L_rear * np.tan(self.steer), WHEELBASE)
        R = L_rear / np.sin(beta)
        print("회전 반경: ", R)

        if np.abs(R) == np.inf:
            vel = self.vel_right
        else:
            # diff-drive robot model
            # vel = self.vel_right * (R / (R+TRACK/2))
            # print('오른쪽 바퀴 속도 :', self.vel_right, '가운데 바퀴 속도: ',vel)
            
            # ackerman steering model
            delta_right = np.arctan2(WHEELBASE, WHEELBASE/np.tan(self.steer) + TRACK/2)
            
            print('중앙 조향:', self.steer, '오른쪽 조향:', delta_right)
            vel = (R / WHEELBASE) * np.sin(delta_right) * (self.vel_right)
            
        # print('처음:', self.x)
        self.x = self.x + vel * np.cos(self.psi + beta) * dt
        self.y = self.y + vel * np.sin(self.psi + beta) * dt
        self.psi = normalize_angle(self.psi + dt * (vel / WHEELBASE) * np.cos(beta) * np.tan(self.steer))
        print("current pose:", (self.x, self.y, self.psi, vel))
        
        self.imu_x = self.imu_x + vel * np.cos(self.imu_psi + beta) * dt
        self.imu_y = self.imu_y + vel * np.sin(self.imu_psi + beta) * dt
        
        #self.data_plot.update([self.psi, self.imu_psi])
        
        # ROS Publish
        dead_reckoning_result = self.make_odometry_msg(self.x, self.y, self.psi)
        dead_reckoning_result_imu = self.make_odometry_msg(self.imu_x, self.imu_y , self.imu_psi)
        self.dead_reckoning_pub.publish(dead_reckoning_result)
        self.dead_reckoning_imu_pub.publish(dead_reckoning_result_imu)
        
        print("")
        return
    
    def make_odometry_msg(self, x, y, yaw):
        odometry_msg = Odometry(header = Header(frame_id='odom', stamp=rospy.Time.now()))
        
        odometry_msg.pose.pose.position.x = x
        odometry_msg.pose.pose.position.y = y
        
        odometry_msg.pose.pose.orientation.x, odometry_msg.pose.pose.orientation.y, \
        odometry_msg.pose.pose.orientation.z, odometry_msg.pose.pose.orientation.w = quaternion_from_euler(0, 0, yaw)
        
        return odometry_msg
    
def main():
    node = DeadReckoning()
    
    try:
        rospy.spin()
        
    except KeyboardInterrupt:
        pass
        
if __name__ == '__main__':
    main()