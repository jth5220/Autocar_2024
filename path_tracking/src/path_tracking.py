#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from control.stanley import Stanley
from control.longitunial_control import RiseTimeImprovement

import rospy
from geometry_msgs.msg import PoseArray, Point, Vector3, TwistWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header, ColorRGBA, Float32, Int8, String,Bool
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDrive

from collections import deque

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class LowPassFilter():
    def __init__(self, alpha=0.3):
        self.v_prev = 0.0
        self.alpha = alpha
        return
    
    def update(self, v):
        self.v_prev = v * self.alpha + self.v_prev * (1-self.alpha)
        return self.v_prev

    
class PathTracking():
    def __init__(self):
        self.path = None

        self.car_x = None
        self.car_y = None
        self.car_yaw = None
        self.car_speed = 0
        self.obstacles = []
        self.gear = 0
        self.target_steer = 0
        self.driving_mode = 'normal_driving'
        
        self.max_speed_way = 2.5
        self.min_speed_way = 2.0

        self.delivery_path = False
        self.is_mission_finished = False
        # steer 필터들
        self.lpf = LowPassFilter(alpha=0.3)
        
        # self.max_speed_mission = {'obstacle_avoiding':  1.8, 'parking': 2.0, 'delivery_start': 1.8, 'delivery_finish': 0.85}
        # self.min_speed_mission = {'obstacle_avoiding':  1.2, 'parking': 1.2, 'delivery_start': 1.5, 'delivery_finish': 0.75}
                
        self.max_speed_mission = {'obstacle_avoiding':  1.8, 'parking': 2.0, 'delivery_start': 1.8, 'delivery_finish': 1.6}
        self.min_speed_mission = {'obstacle_avoiding':  1.2, 'parking': 1.2, 'delivery_start': 1.5, 'delivery_finish': 1.5}
        
        self.k_mission = {'normal_driving':1.0,'intersect':1.0, 'obstacle_avoiding':  1.5, 'parking': 1.0, 'delivery_start': 1.0, 'delivery_finish': 1.0,'lane_change':1.0} # normal_driving':1.2
        self.ld_mission = {'normal_driving':0.7,'intersect':0.8,'obstacle_avoiding':  0.3, 'parking': 0.7, 'delivery_start': 0.5, 'delivery_finish': 0.5,'lane_change':0.5} #'normal_driving':0.7
        # ['normal_driving', 'obstacle_avoiding', 'intersect', 'parking', 'delivery_start','delivery_finish','lane_change']
        
        # 

        ### Stanley ### (CARLA)
        # self.controller = Stanley(k=0.8, ks=0.5, kd=0, L=0.7,
        #                            k_long=3.0, scaling_factor=0.5, max_speed=4.0, min_speed=2.5)
        
        self.controller = Stanley(k=0.8, ks=0.5, kd=0, L=1.3, k_yaw=1.0,
                                   ld_long=1.5, ld_lat=0.5, scaling_factor=np.radians(50), max_speed=1.5, min_speed=1.0)
        
        self.long_controller = RiseTimeImprovement(kp=1.0, ki=0.0, kd=0.0, brake_gain=60)

        #ROS
        rospy.init_node('path_tracking')

        self.speed_maxmin_sub = rospy.Subscriber('/speed_maxmin', Vector3, self.callback_speed_maxmin_way)
        self.driving_mode_sub = rospy.Subscriber("/driving_mode", String, self.callback_driving_mode)
        self.path_sub = rospy.Subscriber("/local_path", Path, self.callback_local_path)
        self.gear_sub = rospy.Subscriber("/gear", Int8, self.callback_gear)

        self.location_sub = rospy.Subscriber("/location", Odometry, self.callback_location)
        # self.speed_sub = rospy.Subscriber("/cur_speed", Float32, self.callback_speed)
        self.speed_sub = rospy.Subscriber("/ublox_gps/fix_velocity", TwistWithCovarianceStamped, self.callback_speed)


        self.mission_path_sub = rospy.Subscriber('/delivery_path_start', Bool, self.callback_delivery_path_start)
        self.is_mission_finished_sub = rospy.Subscriber("/is_mission_finished", Bool, self.callback_mission_finished)


        self.cmd_pub = rospy.Publisher("/erp_command", AckermannDrive, queue_size=10)

        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.callback_path_tracking)
        return
    
    def callback_speed_maxmin_way(self, speed_maxmin_msg):
        self.max_speed_way = speed_maxmin_msg.x
        self.min_speed_way = speed_maxmin_msg.y
        return

    def callback_delivery_path_start(self, path_msg):
        if path_msg.data != self.delivery_path:
            self.max_speed_mission['delivery_finish'] = 0.85
            self.min_speed_mission['delivery_finish'] = 0.75
            self.delivery_path = path_msg.data
            print("배달속도 줄어듦")

    
    def callback_mission_finished(self, path_msg):
        self.is_mission_finished = path_msg.data
        
        if self.is_mission_finished and self.driving_mode == 'delivery_finish':
            self.max_speed_mission['delivery_finish'] = 3.5
            self.min_speed_mission['delivery_finish'] = 3.0
            
        elif self.is_mission_finished and self.driving_mode == 'parking':
            self.max_speed_mission['parking'] = 3.5
            self.min_speed_mission['parking'] = 3.0
            self.k_mission['parking'] = 1.2 
            self.ld_mission['parking'] = 0.7

    def callback_driving_mode(self, driving_mode_msg):
        self.driving_mode = driving_mode_msg.data
        self.controller.max_speed = self.max_speed_mission.get(self.driving_mode, self.max_speed_way)
        self.controller.min_speed = self.min_speed_mission.get(self.driving_mode, self.min_speed_way)
        self.controller.k = self.k_mission[self.driving_mode]
        self.controller.ld_lat = self.ld_mission[self.driving_mode]
        
        if self.driving_mode == 'normal_driving':
            self.controller.k_yaw = 0.8
        else:
            self.controller.k_yaw = 1.0

    def callback_path_tracking(self, event):
        if self.car_x is None or self.path is None:
            return
         # ### Stanley ###
        if self.gear == 2:
            self.controller.L = 1.3
        else:
            self.controller.L = 0.7
        
        target_steer, target_speed = self.controller.feedback(self.car_x, self.car_y, self.car_yaw, self.car_speed, self.path['x'], self.path['y'], self.path['yaw'], self.gear)
        print("원래 조향각: ", target_steer)
        print("yaw_k: ",self.controller.k_yaw)
        if self.driving_mode == 'normal_driving':
            # target_steer = target_steer* np.abs(self.tanh_scaling(target_steer,0.8))
            target_steer *= 0.6
            # self.lpf.alpha = 0.3

            # target_steer = self.lpf.update(target_steer)

        elif self.driving_mode == 'intersect' or self.driving_mode == 'lane_change':
            target_steer *= 0.6
            # self.lpf.alpha = 0.8        
        
        elif self.is_mission_finished and self.driving_mode == 'parking':
            target_steer *= 0.4

        else:
            target_steer *= 0.95
            self.lpf.alpha = 1

        print("max:", self.controller.max_speed ,"min:",self.controller.min_speed)
        # target_steer = np.radians(target_steer) # CARLA
        
        # target_steer = self.lpf.update(target_steer)
    
        # ROS Publish

        cmd_msg = AckermannDrive()
        cmd_msg.steering_angle = target_steer # 최종 입력 조향각
        
        # # CARLA
        # cmd_msg.speed = target_speed # 최종 입력 속도
        
        # print("target_brake: ", target_brake )

        # ERP
        if self.gear == 0:
            final_speed, target_brake = self.long_controller.update(target_speed, self.car_speed)

        elif self.gear == 2:
            if abs(target_speed) - self.car_speed > 1.0:
                final_speed = -2.5
            else:
                final_speed = target_speed
            target_brake = 0

            if self.driving_mode == 'parking':
                self.k_mission['parking'] = 1.5
                self.ld_mission['parking'] = 0.0

        elif self.gear == 1:
            # if self.car_speed > 0.1:
            target_brake = 200
            final_speed = 0
        else:
            pass

        # print("final_target_brake: ", target_brake )
        cmd_msg.speed = final_speed # 최종 입력 속도
        cmd_msg.jerk = target_brake # 최종 입력 브레이크
        self.cmd_pub.publish(cmd_msg)

        print("조향각: ", cmd_msg.steering_angle)
        print("목표 속도: ", cmd_msg.speed)
        print("브레이크: ", cmd_msg.jerk)
        print("#"*30)

        return
    
    def tanh_scaling(self, steering_value, scailing_factor):
        correction_factor = np.tanh(steering_value * scailing_factor)
        return correction_factor

    def callback_gear(self, gear_msg):
        self.gear = gear_msg.data
        return
    
    def callback_speed(self, speed_msg):
        self.car_speed = np.sqrt(speed_msg.twist.twist.linear.x **2 + speed_msg.twist.twist.linear.y**2)
        # self.car_speed = speed_msg.data
        print("현재 속도: ", self.car_speed)
        return

    def callback_location(self, location_msg):
        self.car_x, self.car_y = location_msg.pose.pose.position.x, location_msg.pose.pose.position.y
        self.car_yaw = euler_from_quaternion([location_msg.pose.pose.orientation.x, location_msg.pose.pose.orientation.y,\
                                        location_msg.pose.pose.orientation.z, location_msg.pose.pose.orientation.w])[2] 
        return
    
    def callback_local_path(self, path_msg):
        # self.path = self.get_path(path_msg)
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
    