#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt

import pyproj
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import rospy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl

import csv


def latlon_to_utm(lat, lon):
    # proj = '+proj=utm +zone=52 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
    # latlon_to_utm = pyproj.Proj(proj, preserve_units=True)
    latlon_to_utm = pyproj.Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
    
    return latlon_to_utm(lon, lat)

def normalize_angle(angle):
    
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
        
    return angle

class LowPassFilter():
    def __init__(self, alpha=0.3):
        self.v_prev = 0.0
        self.alpha = alpha
        return
    
    def update(self, v):
        self.v_prev = v * self.alpha + self.v_prev * (1-self.alpha)
        return self.v_prev
    
class DeadReckoning():
    def __init__(self):
        self._set_attributes()
        self._set_ros()
        return
    
    def _set_attributes(self):
        self.init_pose_set = False
        self.lpf = LowPassFilter(alpha=0.3)

        self.dt = 0.1 # [s]

        theta = np.radians(-1.4440643432812905)
        self.RM_offset = np.array([
                                   [np.cos(theta), np.sin(-theta), 0, 0],
                                   [np.sin(theta), np.cos(theta) , 0, 0],
                                   [0,             0,              1, 0],
                                   [0,             0,              0, 1]
                                ])
        
        # [x, y, yaw, v]
        self.init_pose = np.zeros((4, 1)) 
        self.car_pose_utm = np.zeros((4, 1))
        self.car_pose_dr = np.zeros((4, 1))
        self.car_pose_dr_offset = np.zeros((4, 1))
        self.steer = 0.0

        self.wheelbase = 1.566 # [m]
        self.l_r = self.wheelbase * 1.0 # [m]
        self.l_f = self.wheelbase * 0.0 # [m]

        # self.hx_utm = []
        # self.hx_dr = []
        return
    
    def _set_ros(self):
        # ROS
        rospy.init_node('dead_reckoning')

        self.gps_sub = rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.callback_gps)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.callback_imu)
        self.speed_sub = rospy.Subscriber('/encoder/speed', Float32, self.callback_speed)
        self.cmd_sub = rospy.Subscriber('/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl, self.callback_cmd)
        

        self.pose_utm_pub = rospy.Publisher('/pose_utm', Odometry, queue_size=10)
        self.pose_dr_pub = rospy.Publisher('/pose_dr', Odometry, queue_size=10)
        self.timer_dr = rospy.Timer(rospy.Duration(0.1), self.callback_dead_reckoning)
        return
    
    def callback_dead_reckoning(self, event):
        beta = np.arctan2((self.l_r * np.tan(self.steer)), (self.l_r + self.l_f))
        theta = normalize_angle(self.car_pose_dr[2, 0] + beta)
        print('check:', self.steer, beta)

        u = np.array([[theta],
                      [self.car_pose_dr[3, 0]]]) # [yaw+beta, v]
        
        self.car_pose_dr = self.motion_model(self.car_pose_dr, u)
        self.car_pose_dr_offset = self.RM_offset @ self.car_pose_dr

        print("비교 시작")
        print(self.car_pose_dr_offset) # Dead reckoning
        print(self.car_pose_utm - self.init_pose) # UTM
        print()

        car_pose_utm = self.car_pose_utm # Dead reckoning - offseted
        car_pose_dr = self.car_pose_dr_offset + self.init_pose

        pose_utm = Odometry()
        pose_utm.header.frame_id = 'utm'
        pose_utm.pose.pose.position.x, pose_utm.pose.pose.position.y = car_pose_utm[0,0], car_pose_utm[1,0]
        pose_utm.pose.pose.orientation.x, pose_utm.pose.pose.orientation.y,\
        pose_utm.pose.pose.orientation.z, pose_utm.pose.pose.orientation.w = quaternion_from_euler(0, 0, car_pose_utm[2,0])
        self.pose_utm_pub.publish(pose_utm)

        pose_dr = Odometry()
        pose_dr.header.frame_id = 'utm'
        pose_dr.pose.pose.position.x, pose_dr.pose.pose.position.y = car_pose_dr[0,0], car_pose_dr[1,0]
        pose_dr.pose.pose.orientation.x, pose_dr.pose.pose.orientation.y,\
        pose_dr.pose.pose.orientation.z, pose_dr.pose.pose.orientation.w = quaternion_from_euler(0, 0, car_pose_dr[2,0])
        self.pose_dr_pub.publish(pose_dr)

        # car_pose_utm_ = self.car_pose_utm - self.init_pose
        # self.hx_utm.append([car_pose_utm_[0,0], car_pose_utm_[1,0]])
        # self.hx_dr.append([self.car_pose_dr[0,0], self.car_pose_dr[1,0]])
        return
    
    def motion_model(self, x, u):
        # x = [x,y,yaw,v].T
        # u = [yaw+beta, v]
        F = np.array([[1.0, 0, 0, 0],
                      [0, 1.0, 0, 0],
                      [0, 0, 1.0, 0],
                      [0, 0, 0, 0.0]])

        B = np.array([[0.0, self.dt * np.cos(u[0,0])],
                      [0.0, self.dt * np.sin(u[0,0])],
                      [0.0, 0.0],
                      [0.0, 1.0]])

        x = F @ x + B @ u
        return x
    
    def callback_cmd(self, cmd_msg):
        self.steer = self.lpf.update(-cmd_msg.steer)
        return
    
    def callback_gps(self, gps_msg):
        x_utm, y_utm = latlon_to_utm(gps_msg.latitude, gps_msg.longitude)

        # Set the initial pose
        if not self.init_pose_set:
            self.init_pose[0, 0] = x_utm
            self.init_pose[1, 0] = y_utm
            self.init_pose_set = True  # Update the flag
        
        # Update measurment - UTM position
        self.car_pose_utm[0, 0] = x_utm
        self.car_pose_utm[1, 0] = y_utm

        return
    
    def callback_imu(self,imu_msg):
        yaw = normalize_angle(2.0 * np.arctan2(imu_msg.orientation.z, imu_msg.orientation.w))

        # Update measurment - yaw
        self.car_pose_utm[2, 0] = yaw
        self.car_pose_dr[2, 0] = yaw
        
    def callback_speed(self, speed_msg):
        speed = speed_msg.data
         # Update measurment - speed
        self.car_pose_utm[3, 0] = speed
        self.car_pose_dr[3, 0] = speed
        return
    
if __name__ == "__main__":
    try:
        # ROS
        dr = DeadReckoning()
        rospy.spin()
        
        # filename_utm = 'utm.csv'
        # filename_dr = 'dr.csv'
        
        # print(dr.hx_dr)
        # print(dr.hx_utm)

        # with open(filename_utm, mode='w', newline='') as file:
        #     writer = csv.writer(file)
        #     writer.writerow(['x', 'y'])  # 헤더 작성
        #     writer.writerows(dr.hx_utm) 

        # with open(filename_dr, mode='w', newline='') as file:
        #     writer = csv.writer(file)
        #     writer.writerow(['x', 'y'])  # 헤더 작성
        #     writer.writerows(dr.hx_dr)

    except rospy.ROSInterruptException:
        pass