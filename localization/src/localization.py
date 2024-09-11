#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pyproj
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from collections import deque

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import rospy
import message_filters
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, QuaternionStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int16, Int32, Float32MultiArray
from ackermann_msgs.msg import AckermannDrive
# from filterpy.kalman import KalmanFilter

def latlon_to_utm(lat, lon):
    proj = '+proj=utm +zone=52 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
    latlon_to_utm = pyproj.Proj(proj, preserve_units=True)
    return latlon_to_utm(lon, lat)

def normalize_angle(angle):
    
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
        
    return angle

class Localization():
    def __init__(self):
        self.location = Odometry()
        self.location.header.frame_id = 'utm'

        self.location_corrected = Odometry()
        self.location_corrected.header.frame_id = 'utm'
        self.location_corrected.pose.pose.orientation.x, self.location_corrected.pose.pose.orientation.y, \
        self.location_corrected.pose.pose.orientation.z, self.location_corrected.pose.pose.orientation.w = 0,0,0,1
        
        self.location_long_corrected = Odometry()
        self.location_long_corrected.header.frame_id = 'utm'
        
        # self.location_dead = Odometry()
        # self.location_dead.header.frame_id = 'utm'
        
        # self.location_kalman = Odometry()
        # self.location_kalman.header.frame_id = 'utm'
        
        self.yaw_offset = 0 # radians
        self.global_yaw = 0
        
        #횡방향 관련 초기값
        self.lateral_offset = (0,0) # meters
        self.lateral_error = 0
    
        #종방향 관련 초기값
        self.longitudinal_error = 0
        self.longitudinal_offset = (0,0)
        self.is_longitudinal_error_calculated = True
        self.closest_stopline = 0
        self.closest_stopline_prev = 0
        
        # 방향정보 관련 초기값
        self.location_history = deque(maxlen=5)
        self.gps_mean_yaw = 0
        self.init_yaw = False
        
        self.closest_node_id = 0
        self.car_pos = None
        self.path_curvature = 0
        self.path_first_cte = 0
        self.path_mid_cte = 0
        
        # 현재 속도 및 조향
        self.speed = 0
        self.steering_angle = 0
        
        #데드레코딩 초기값
        self.wheelbase = 1.04
        self.dead_x = 0.0
        self.dead_y = 0.0
        self.dead_reckoning_initialized = False
        
        # 칼만 필터 초기화
        # self.kf = KalmanFilter(dim_x=5, dim_z=3)  # x, y, yaw, velocity, steering_angle
        # self.kf.F = np.eye(5)  # 초기화 시 후에 업데이트
        # self.kf.H = np.array([[1, 0, 0, 0, 0],
        #                       [0, 1, 0, 0, 0],
        #                       [0, 0, 1, 0, 0]])
        # self.kf.P *= 10  # 불확실성을 증가
        # self.kf.R = np.diag([5, 5, 0.1])  # GPS의 관측 노이즈
        # self.kf.Q = np.diag([0.1, 0.1, 0.1, 0.1, 0.1])  # 프로세스 노이즈
                
        # ROS
        rospy.init_node('localization')
        
        #Erp command 불러오기
        self.cmd_sub = rospy.Subscriber("/erp_command", AckermannDrive, self.callback_cmd)
        
        #엔코더 속도 불러오기
        self.encoder_vel = rospy.Subscriber("/data/encoder_vel",Odometry, self.callback_encoder_vel)
        
        # gps, imu sub
        self.gps_sub = rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.callback_gps)
        self.imu0_sub = rospy.Subscriber('/imu/data', Imu, self.callback_imu)
        # self.gps_sub = rospy.Subscriber('/carla/ego_vehicle/gnss', NavSatFix, self.callback_gps)
        # self.imu0_sub = rospy.Subscriber('/carla/ego_vehicle/imu', Imu, self.callback_imu)
        
        #초기 yaw 설정
        self.init_orientation_sub = rospy.Subscriber('/initial_global_pose', PoseWithCovarianceStamped, self.callback_init_orientation)
        
        #global local 횡방향 종방향 에러 sub
        self.local_cte_sub = message_filters.Subscriber('/ct_error_local', Float32)
        self.global_sub = message_filters.Subscriber('/ct_error_global', Float32)
        self.cte = message_filters.ApproximateTimeSynchronizer([self.local_cte_sub,self.global_sub],queue_size=10,slop=0.1,allow_headerless=True)
        self.cte.registerCallback(self.callback_cte)
        

        self.local_ate_sub = message_filters.Subscriber('/at_error_local', Float32)
        self.global_ate_sub = message_filters.Subscriber('/at_error_global', Float32)
        self.ate = message_filters.ApproximateTimeSynchronizer([self.local_ate_sub,self.global_ate_sub],queue_size=10,slop=0.1,allow_headerless=True)
        self.ate.registerCallback(self.callback_ate)
        
        # 제일 가까운 node id
        self.closest_node_id_sub = rospy.Subscriber('/closest_node_id',Int32,self.callback_closest_node_id)
        
        # 경로의 곡률 sub
        self.path_info_sub = rospy.Subscriber('/path_info',Float32MultiArray,self.callback_path_info)
        
        # 제일 가까운 정지선 sub
        self.closest_stopline_sub = rospy.Subscriber('/closest_stopline',Int16,self.callback_closest_stopline)
        
        # 보정안된 위치와 보정된 위치 pub
        self.location_no_correction_pub = rospy.Publisher('/location_not_corrected', Odometry, queue_size=10)
        self.location_long_corrected_pub = rospy.Publisher('/location_long_corrected', Odometry,queue_size=10)
        self.location_corrected_pub = rospy.Publisher('/location_corrected', Odometry, queue_size=10)
        # self.location_dead_pub = rospy.Publisher('/location_dead', Odometry, queue_size=10)
        # self.location_kalman_pub = rospy.Publisher('/location_kalman', Odometry, queue_size=10)
        
        # 최종 보정된 종방향 횡방향 에러 pub
        self.lateral_error_pub = rospy.Publisher('/lateral_error', Float32, queue_size=10)
        self.longitudinal_error_pub = rospy.Publisher('/longitudinal_error', Float32, queue_size=10)

        # 0.1초마다 callback함수 계산해주기
        self.timer_location_publish = rospy.Timer(rospy.Duration(0.1), self.callback_timer_location_pub)
    
    def initialize_dead_reckoning(self):
        """Initialize Dead Reckoning with corrected location."""
        self.dead_x = self.location_corrected.pose.pose.position.x
        self.dead_y = self.location_corrected.pose.pose.position.y
        _, _, self.dead_yaw = euler_from_quaternion([
            self.location_corrected.pose.pose.orientation.x,
            self.location_corrected.pose.pose.orientation.y,
            self.location_corrected.pose.pose.orientation.z,
            self.location_corrected.pose.pose.orientation.w
        ])
        self.dead_reckoning_initialized = True

    def callback_timer_location_pub(self, event):
        self.location_no_correction_pub.publish(self.location)
        self.location_corrected_pub.publish(self.location_corrected)
        self.location_long_corrected_pub.publish(self.location_long_corrected)
        # self.location_dead_pub.publish(self.location_dead)
        # self.location_kalman_pub.publish(self.location_kalman)
                
        lateral_error_msg = Float32()
        lateral_error_msg.data = self.lateral_error
        self.lateral_error_pub.publish(lateral_error_msg)
        
        longitudinal_error_msg = Float32()
        longitudinal_error_msg.data = self.longitudinal_error
        self.longitudinal_error_pub.publish(longitudinal_error_msg)
        
        # if not self.dead_reckoning_initialized and self.init_yaw:
        #     self.initialize_dead_reckoning()
    
    
    def callback_gps(self, gps_msg):
        self.location.pose.pose.position.x, self.location.pose.pose.position.y = latlon_to_utm(gps_msg.latitude, gps_msg.longitude)
        
        # self.location_history.append((self.location.pose.pose.position.x, self.location.pose.pose.position.y))
        # location_xs = np.array([point[0] for point in self.location_history])
        # location_ys = np.array([point[1] for point in self.location_history])
        
        # if len(self.location_history) == 5:
        #     dxs = np.diff(location_xs)
        #     dys = np.diff(location_ys)
        #     slopes_np = np.arctan2(dys , dxs)
        #     self.gps_mean_yaw = np.mean(slopes_np)
    
        
        self.location_long_corrected.pose.pose.position.x = self.location.pose.pose.position.x - self.longitudinal_offset[0]
        self.location_long_corrected.pose.pose.position.y = self.location.pose.pose.position.y - self.longitudinal_offset[1]
       
        self.location_corrected.pose.pose.position.x = self.location_long_corrected.pose.pose.position.x - self.lateral_offset[0]
        self.location_corrected.pose.pose.position.y = self.location_long_corrected.pose.pose.position.y - self.lateral_offset[1]
                
        # gps_x, gps_y = latlon_to_utm(gps_msg.latitude, gps_msg.longitude)
        # z = np.array([gps_x, gps_y, self.global_yaw])
        # self.kf.update(z)
        
    def callback_imu(self,imu_msg):
        local_yaw = euler_from_quaternion([imu_msg.orientation.x, imu_msg.orientation.y,\
                                          imu_msg.orientation.z, imu_msg.orientation.w])[2]
        global_yaw = local_yaw + self.yaw_offset
        
        self.global_yaw = normalize_angle(global_yaw)
        self.location.pose.pose.orientation.x, self.location.pose.pose.orientation.y, \
        self.location.pose.pose.orientation.z, self.location.pose.pose.orientation.w = quaternion_from_euler(0, 0, self.global_yaw)

        self.location_corrected.pose.pose.orientation = self.location.pose.pose.orientation
        self.location_long_corrected.pose.pose.orientation = self.location.pose.pose.orientation
        return 
        
    def callback_cmd(self,cmd_msg):
        self.steering_angle = np.radians(cmd_msg.steering_angle)
        self.speed = cmd_msg.speed
        
    def callback_encoder_vel(self, enc_msg):
        #self.speed = enc_msg.twist.twist.linear.x
        # if self.dead_reckoning_initialized:
        #     self.dead_reckoning()
        return
    
    def dead_reckoning(self):
        dt = 0.1
        
        delta_x = self.speed * np.cos(self.global_yaw + self.steering_angle) * dt
        delta_y = self.speed * np.sin(self.global_yaw + self.steering_angle) * dt
        delta_psi = (self.speed / self.wheelbase) * np.tan(self.steering_angle) * dt

        self.dead_x += delta_x
        self.dead_y += delta_y
        self.dead_yaw = normalize_angle(self.dead_yaw + delta_psi)
        
        self.location_dead.pose.pose.position.x, self.location_dead.pose.pose.position.y = self.dead_x, self.dead_y
        self.location_dead.pose.pose.orientation.x, self.location_dead.pose.pose.orientation.y,\
        self.location_dead.pose.pose.orientation.z, self.location_dead.pose.pose.orientation.w = quaternion_from_euler(0, 0, self.global_yaw)

        # # 칼만 필터 예측 단계
        # F = np.array([[1, 0, -self.speed * dt * np.sin(self.dead_yaw), dt * np.cos(self.dead_yaw), 0],
        #               [0, 1, self.speed * dt * np.cos(self.dead_yaw), dt * np.sin(self.dead_yaw), 0],
        #               [0, 0, 1, 0, dt / self.wheelbase],
        #               [0, 0, 0, 1, 0],
        #               [0, 0, 0, 0, 1]])
        # self.kf.F = F
        # u = np.array([0, 0, 0, self.speed, self.steering_angle])
        # self.kf.predict(u)

        # self.location_kalman.pose.pose.position.x = self.kf.x[0]
        # self.location_kalman.pose.pose.position.y = self.kf.x[1]
        # self.location_kalman.pose.pose.orientation.x, self.location_kalman.pose.pose.orientation.y,\
        # self.location_kalman.pose.pose.orientation.z, self.location_kalman.pose.pose.orientation.w = quaternion_from_euler(0, 0, self.kf.x[2])

    
    def callback_init_orientation(self, init_pose_msg):
        global_yaw = euler_from_quaternion([init_pose_msg.pose.pose.orientation.x, init_pose_msg.pose.pose.orientation.y, \
                                           init_pose_msg.pose.pose.orientation.z, init_pose_msg.pose.pose.orientation.w])[2]
       
        local_yaw = euler_from_quaternion([self.location.pose.pose.orientation.x, self.location.pose.pose.orientation.y,\
                                          self.location.pose.pose.orientation.z, self.location.pose.pose.orientation.w])[2]

        self.yaw_offset += global_yaw - local_yaw
    
        #self.init_yaw = True
    
    
    def callback_cte(self, local_cte_msg, global_cte_msg):
        
        if self.car_pos == "vision":
            global_cte = global_cte_msg.data
            local_cte = local_cte_msg.data
            lateral_error = local_cte - global_cte
        
            self.lateral_error = lateral_error
            perpendicular_direction = self.global_yaw + np.pi/2
            self.lateral_offset = (self.lateral_error * np.cos(perpendicular_direction), self.lateral_error * np.sin(perpendicular_direction))

        
        #print(self.lateral_error)
        
    
    def callback_ate(self, local_ate_msg, global_ate_msg):
        global_ate = global_ate_msg.data
        local_ate = local_ate_msg.data
        
        if 3.1 < local_ate < 7.5 and 0 < global_ate < 25 and self.is_longitudinal_error_calculated:  #base_link에서 차 위치가 (2,0) 으로 시작하기 때문에
            longitudinal_error = local_ate - global_ate
            
            parallel_direction = self.global_yaw
            perpendicular_direction = self.global_yaw + np.pi/2
            #print(parallel_direction)
            self.longitudinal_offset = (longitudinal_error * np.cos(parallel_direction)+self.lateral_error*np.cos(perpendicular_direction),
                                        longitudinal_error * np.sin(parallel_direction)+self.lateral_error*np.sin(perpendicular_direction))
            
            self.is_longitudinal_error_calculated = False
         
    
    def callback_closest_node_id(self, closest_node_id_msg):
        self.closest_node_id = closest_node_id_msg.data
        id = (self.closest_node_id // 10000) % 10
        
        
        if id == 0:
            self.car_pos = "vision"
            if self.path_curvature > 0.6  or self.path_mid_cte > 1.3:
                self.car_pos = "obstacle"
        elif id == 1:
            self.car_pos = "intersect"
        elif id == 2:
            self.car_pos = "mission"
            
        else:
            pass
        
        print(self.car_pos)
        
    def callback_path_info(self,path_info_msg):
        self.path_curvature = path_info_msg.data[0]
        self.path_first_cte = path_info_msg.data[1]
        self.path_mid_cte = path_info_msg.data[2]
        
    def callback_closest_stopline(self,closest_stopline_msg):
        self.closest_stopline = closest_stopline_msg.data
        
        if self.closest_stopline != self.closest_stopline_prev:
            self.is_longitudinal_error_calculated = True
            
        self.closest_stopline_prev = self.closest_stopline
    
if __name__ == "__main__":
    try:
        # ROS
        localization = Localization()
        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass