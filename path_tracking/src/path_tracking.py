#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from control.stanley import Stanley
# from control.purepursuit import PurePursuit
from control.longitunial_control import RiseTimeImprovement

import rospy
from geometry_msgs.msg import PoseArray, Point, Vector3, TwistWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32, Int8, String
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDrive

from tf.transformations import euler_from_quaternion, quaternion_from_euler
    
class PathTracking():
    def __init__(self):
        self.path = None

        self.car_x = None
        self.car_y = None
        self.car_yaw = None
        self.car_speed = 0
        self.obstacles = []
        self.gear = 0

        self.desired_max_speed = { 'normal_driving' : 4.0, 'obstacle_avoiding':  3.5, 'intersect': 4.0, 'parking': 2.5, 'delivery_start': 1.8, 'delivery_finish': 1.8 }
        self.desired_min_speed = { 'normal_driving' : 2.5, 'obstacle_avoiding':  2.0, 'intersect': 2.5, 'parking': 2.0, 'delivery_start': 1.0, 'delivery_finish': 1.0 }

        ### Stanley ###
        self.controller = Stanley(k=0.8, ks=0.5, kd=0, L=0.7,
                                   k_long=3.0, scaling_factor=0.5, max_speed=4.0, min_speed=2.5)

        ### Pure Pursuit ###
        # self.controller = PurePursuit(k=2.0, L=1.3, ks=0.5)
        
        self.long_controller = RiseTimeImprovement(kp=4.0, ki=0.0, kd=0.0)

        #ROS
        rospy.init_node('path_tracking')

        self.driving_mode_sub = rospy.Subscriber("/driving_mode", String, self.callback_driving_mode)
        self.path_sub = rospy.Subscriber("/local_path", Path, self.callback_local_path)
        self.gear_sub = rospy.Subscriber("/gear", Int8, self.callback_gear)

        self.location_sub = rospy.Subscriber("/location_corrected", Odometry, self.callback_location)

        # self.speed_sub = rospy.Subscriber("/ublox_gps/fix_velocity", TwistWithCovarianceStamped, self.callback_speed)
        self.speed_sub = rospy.Subscriber("/encoder/speed", Float32, self.callback_speed)

        self.cmd_pub = rospy.Publisher("/erp_command", AckermannDrive, queue_size=10)

        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.callback_path_tracking)
        return
    
    def callback_driving_mode(self, driving_mode_msg):
        self.driving_mode = driving_mode_msg.data
        self.controller.max_speed = self.desired_max_speed[self.driving_mode]
        self.controller.min_speed = self.desired_min_speed[self.driving_mode]
        

    def callback_path_tracking(self, event):
        if self.car_x is None or self.path is None:
            return
         # ### Stanley ###
        if self.gear == 2:
            self.controller.L = 1.3
        else:
            self.controller.L = 0.7
        
        print("max:", self.controller.max_speed ,"min:",self.controller.min_speed)
        target_steer, target_speed = self.controller.feedback(self.car_x, self.car_y, self.car_yaw, self.car_speed, self.path['x'], self.path['y'], self.path['yaw'], self.gear)
        target_steer = np.radians(target_steer) # CARLA

        if self.gear == 1:
            target_speed = 0
            target_steer = 0
            
            if self.car_speed > 0.1:
                target_brake = 100
            else: target_brake = 0

        else:
            target_brake = 0

        cmd_msg = AckermannDrive()
        cmd_msg.steering_angle = target_steer # 최종 입력 조향각
        cmd_msg.speed = target_speed # 최종 입력 속도
        cmd_msg.jerk = target_brake # 최종 입력 브레이크
        self.cmd_pub.publish(cmd_msg)

        print("조향각: ", np.degrees(cmd_msg.steering_angle))
        print("목표 속도: ", cmd_msg.speed)
        print("브레이크: ", cmd_msg.jerk)
        print("#"*30)

        return
    
    def callback_gear(self, gear_msg):
        self.gear = gear_msg.data
        return
    
    def callback_speed(self, velocity_msg):
        # self.car_speed = np.sqrt(velocity_msg.twist.twist.linear.x **2 + velocity_msg.twist.twist.linear.y**2)
        self.car_speed = velocity_msg.data
        # print("현재 속도: ", self.car_speed)
        return

    def callback_location(self, location_msg):
        self.car_x, self.car_y = location_msg.pose.pose.position.x, location_msg.pose.pose.position.y
        self.car_yaw = euler_from_quaternion([location_msg.pose.pose.orientation.x, location_msg.pose.pose.orientation.y,\
                                        location_msg.pose.pose.orientation.z, location_msg.pose.pose.orientation.w])[2] 
        return
    
    def callback_local_path(self, path_msg):
        path = {'x':[], 'y':[], 'yaw':[]}

        # path_msg가 Path일 때
        for wp in path_msg.poses:
            path['x'].append(wp.pose.position.x)
            path['y'].append(wp.pose.position.y)

            yaw = euler_from_quaternion([wp.pose.orientation.x, wp.pose.orientation.y,
                                         wp.pose.orientation.z, wp.pose.orientation.w])[2]
            path['yaw'].append(yaw)

        self.path = path
        return
    
    def make_marker(self, point):
        marker = Marker()
        marker.header.frame_id = "utm"  # 적절한 프레임 ID로 설정
        marker.header.stamp = rospy.Time.now()
        marker.ns = ""
        marker.id = 999999
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # 좌표 설정
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # 크기 설정
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        # 색상 설정 (파란색)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        # Marker 생존 시간 설정
        marker.lifetime = rospy.Duration()
    
        return marker
    
if __name__ == "__main__":
    try:
        # ROS
        path_tracking = PathTracking()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    