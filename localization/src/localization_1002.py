#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pyproj
import numpy as np
import time
from scipy.spatial import KDTree
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from collections import deque

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import rospy
import message_filters
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, QuaternionStamped, PoseArray
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int16, Int32, Float32MultiArray, String

CORRECT_RATE = 1 # second

class Localization():
    def __init__(self):
        self.gps_pose = Odometry()
        self.gps_pose.header.frame_id = 'utm'

        self.location_corrected = Odometry()
        self.location_corrected.header.frame_id = 'utm'
        self.location_corrected.pose.pose.orientation.x, self.location_corrected.pose.pose.orientation.y, \
        self.location_corrected.pose.pose.orientation.z, self.location_corrected.pose.pose.orientation.w = 0,0,0,1
        
        self.location_long_corrected = Odometry()
        self.location_long_corrected.header.frame_id = 'utm'
        
        self.location = Odometry()
        self.location.header.frame_id = 'utm'
        
        self.yaw_offset = 0 # radians
        self.global_yaw = 0
        
        #횡방향 관련 초기값
        self.local_cte = None
        self.lateral_offset = (0,0) # meters
        self.lateral_error = 0
        self.prev_time = None
        
        #종방향 관련 초기값
        self.stopline_dict = {}
        self.local_ate = 0
        self.longitudinal_error = 0
        self.longitudinal_offset = (0,0)
        self.is_longitudinal_error_calculated = True
        self.closest_stopline_prev = 0
        
        # waypoint 초기값
        self.closest_waypoints = []
        self.global_waypoints = []
        self.closest_wp_idx = None
   
        self.driving_mode = None

        
        # ROS
        rospy.init_node('localization')
        
        # gps, imu sub
        self.gps_sub = rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.callback_gps)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.callback_imu)
        # self.gps_sub = rospy.Subscriber('/carla/ego_vehicle/gnss', NavSatFix, self.callback_gps)
        # self.imu_sub = rospy.Subscriber('/carla/ego_vehicle/imu', Imu, self.callback_imu)
        
        # 현재 주행 모드 가져오기
        self.driving_mode_sub = rospy.Subscriber('/driving_mode', String, self.callback_driving_mode)
        
        #초기 yaw 설정
        self.init_orientation_sub = rospy.Subscriber('/initial_global_pose', PoseWithCovarianceStamped, self.callback_init_orientation)
        
        # global path planning waypoint 정보
        self.closest_waypoints_sub = rospy.Subscriber('/global_closest_waypoints', PoseArray, self.callback_closest_waypoints)
        self.waypoints_sub = rospy.Subscriber('/global_waypoints', PoseArray, self.callback_global_waypoints)
        self.closest_wp_sub = rospy.Subscriber('/closest_wp_idx', Int32, self.callback_closest_wp_idx)
        
        # 정지선 위치
        self.stoplines_sub = rospy.Subscriber('/stoplines', MarkerArray, self.callback_stopline)
        
        #global local 횡방향 종방향 에러 sub
        self.local_cte_sub = rospy.Subscriber('/ct_error_local', Float32, self.callback_cte)
        self.local_ate_sub = rospy.Subscriber('/at_error_local', Float32, self.callback_ate)

        
        # dead reckoning 위치 sub
        self.pose_dr_sub = rospy.Subscriber('/pose_dr', Odometry, self.callback_dr)
        
        # 보정안된 위치와 보정된 위치 pub
        self.location_no_correction_pub = rospy.Publisher('/location_not_corrected', Odometry, queue_size=10)
        # self.location_long_corrected_pub = rospy.Publisher('/location_long_corrected', Odometry,queue_size=10)
        self.location_corrected_pub = rospy.Publisher('/location_corrected', Odometry, queue_size=10)

        self.location_pub = rospy.Publisher('/location',Odometry, queue_size=10)
        
        
        # 최종 보정된 종방향 횡방향 에러 pub
        self.lateral_error_pub = rospy.Publisher('/lateral_error', Float32, queue_size=10)
        self.longitudinal_error_pub = rospy.Publisher('/longitudinal_error', Float32, queue_size=10)

        # 0.1초마다 callback함수 계산해주기
        self.timer_location_publish = rospy.Timer(rospy.Duration(0.1), self.callback_timer_location_pub)
    
    
    def callback_timer_location_pub(self, event):
        location_gps = (self.gps_pose.pose.pose.position.x, self.gps_pose.pose.pose.position.y)

        if self.global_waypoints:
            global_cte = get_cte(location_gps,self.global_waypoints,self.closest_wp_idx)
        
        if self.driving_mode == "normal_driving" :
            #횡방향 위치 보정
            # global_cte = get_cte(location,self.global_waypoints,self.closest_wp_idx)
            local_cte = self.local_cte
            if local_cte is not None:
                lateral_error = local_cte - global_cte
                self.lateral_error = lateral_error
                        
                perpendicular_direction = self.global_yaw + np.pi/2
                self.lateral_offset = (self.lateral_error * np.cos(perpendicular_direction), self.lateral_error * np.sin(perpendicular_direction))
        
            #종방향 위치 보정
        
        if self.closest_waypoints:
            global_ate, closest_stopline = get_ate(location_gps,self.stopline_dict,self.closest_waypoints)
            
            if closest_stopline != self.closest_stopline_prev:
                self.is_longitudinal_error_calculated = True
            self.closest_stopline_prev = closest_stopline
            
            if 3.1 < self.local_ate < 7.5 and 0 < global_ate < 25 and self.is_longitudinal_error_calculated:  #base_link에서 차 위치가 (2,0) 으로 시작하기 때문에
                longitudinal_error = self.local_ate - global_ate
            
                parallel_direction = self.global_yaw
                perpendicular_direction = self.global_yaw + np.pi/2
                #print(parallel_direction)
                self.longitudinal_offset = (longitudinal_error * np.cos(parallel_direction)+self.lateral_error*np.cos(perpendicular_direction),
                                            longitudinal_error * np.sin(parallel_direction)+self.lateral_error*np.sin(perpendicular_direction))
                
                self.is_longitudinal_error_calculated = False
 
            # print(closest_stopline,self.closest_stopline_prev,self.is_longitudinal_error_calculated)
            
        # self.location_long_corrected.pose.pose.position.x = self.gps_pose.pose.pose.position.x - self.longitudinal_offset[0]
        # self.location_long_corrected.pose.pose.position.y = self.gps_pose.pose.pose.position.y - self.longitudinal_offset[1]
        # self.location_long_corrected.pose.pose.orientation = self.gps_pose.pose.pose.orientation

        self.location_corrected.pose.pose.position.x = self.gps_pose.pose.pose.position.x - self.lateral_offset[0] - self.longitudinal_offset[0]
        self.location_corrected.pose.pose.position.y = self.gps_pose.pose.pose.position.y - self.lateral_offset[1] - self.longitudinal_offset[1]
        self.location_corrected.pose.pose.orientation = self.gps_pose.pose.pose.orientation
        
        if self.driving_mode == 'intersect' and self.driving_mode == 'turnel':
            self.location.pose.pose.position = self.dr_pos
            self.location.pose.pose.orientation = self.dr_orientation
        else:
            self.location.pose.pose.position = self.location_corrected.pose.pose.position
            self.location.pose.pose.orientation = self.location_corrected.pose.pose.orientation
        
        self.location_no_correction_pub.publish(self.gps_pose)
        # self.location_long_corrected_pub.publish(self.location_long_corrected)
        self.location_corrected_pub.publish(self.location_corrected)
        self.location_pub.publish(self.location) 
        
        lateral_error_msg = Float32()
        lateral_error_msg.data = self.lateral_error
        self.lateral_error_pub.publish(lateral_error_msg)
        
        longitudinal_error_msg = Float32()
        longitudinal_error_msg.data = self.longitudinal_error
        self.longitudinal_error_pub.publish(longitudinal_error_msg)
        

        
    def callback_driving_mode(self, mode_msg):
        self.driving_mode = mode_msg.data

    def callback_dr(self, dr_msg):
        self.dr_pos = dr_msg.pose.pose.position
        self.dr_orientation = dr_msg.pose.pose.orientation
        
    def callback_gps(self, gps_msg):
        self.gps_pose.pose.pose.position.x, self.gps_pose.pose.pose.position.y = latlon_to_utm(gps_msg.latitude, gps_msg.longitude)
 
        
    def callback_imu(self,imu_msg):
        local_yaw = euler_from_quaternion([imu_msg.orientation.x, imu_msg.orientation.y,\
                                          imu_msg.orientation.z, imu_msg.orientation.w])[2]
        global_yaw = local_yaw + self.yaw_offset
        
        self.global_yaw = normalize_angle(global_yaw)
        self.gps_pose.pose.pose.orientation.x, self.gps_pose.pose.pose.orientation.y, \
        self.gps_pose.pose.pose.orientation.z, self.gps_pose.pose.pose.orientation.w = quaternion_from_euler(0, 0, self.global_yaw)


    def callback_init_orientation(self, init_pose_msg):
        global_yaw = euler_from_quaternion([init_pose_msg.pose.pose.orientation.x, init_pose_msg.pose.pose.orientation.y, \
                                           init_pose_msg.pose.pose.orientation.z, init_pose_msg.pose.pose.orientation.w])[2]
       
        local_yaw = euler_from_quaternion([self.gps_pose.pose.pose.orientation.x, self.gps_pose.pose.pose.orientation.y,\
                                          self.gps_pose.pose.pose.orientation.z, self.gps_pose.pose.pose.orientation.w])[2]

        self.yaw_offset += global_yaw - local_yaw
    
    def callback_closest_waypoints(self, poses_msg):
        closest_waypoints = []
        for pose in poses_msg.poses:
            x = pose.position.x
            y = pose.position.y
            closest_waypoints.append((x, y))
        self.closest_waypoints = closest_waypoints
        
    def callback_global_waypoints(self,poses_msg):
        global_waypoint = []
        for pose in poses_msg.poses:
            x = pose.position.x
            y = pose.position.y
            global_waypoint.append((x, y))
        self.global_waypoints = global_waypoint
        
    def callback_closest_wp_idx(self, idx_msg):
        self.closest_wp_idx = idx_msg.data
        
    def callback_cte(self, local_cte_msg):
        self.local_cte = local_cte_msg.data
        
    
    def callback_ate(self, local_ate_msg):
        self.local_ate = local_ate_msg.data
    
    def callback_stopline(self,stoplines_msg):
        for stopline in stoplines_msg.markers:
            # marker의 id를 추출
            marker_id = stopline.id
            c1 = (stopline.points[0].x,stopline.points[0].y)
            c2 = (stopline.points[1].x,stopline.points[1].y)
                        
            # id를 키로 하고 좌표 (x, y)를 배열로 저장
            if marker_id not in self.stopline_dict:
                self.stopline_dict[marker_id] = []

            # x, y 좌표를 리스트에 추가
            self.stopline_dict[marker_id].append([c1,c2])
        
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

def get_cte(car_position, waypoints, closest_wp_idx):
        
        try:
            first_node = waypoints[closest_wp_idx]
            second_node = waypoints[closest_wp_idx + 1]
        except IndexError:
            first_node = waypoints[closest_wp_idx - 1]
            second_node = waypoints[closest_wp_idx]
            print('err')
        linear_vector = [(second_node[0] - first_node[0]) , (second_node[1] - first_node[1])]
        slope = linear_vector[1]/linear_vector[0]
                
        intercept = first_node[1] - slope*first_node[0]
        # print(linear_vector,slope,intercept)
        if linear_vector[0] >= 0:
            line_coef = [slope, -1, intercept] # ax-y+b=0
        # elif waypoint 방향 감소: -ax +y -b=0
        else:
            line_coef = [-slope, 1, -intercept] # ax+y+b=0
            
        cross_track_error = (line_coef[0]*car_position[0] + line_coef[1]*car_position[1] + line_coef[2])\
                                / np.sqrt(line_coef[0]**2 + line_coef[1]**2)
        # print(cross_track_error)
        return cross_track_error

def get_ate(car_position, stopline, near_waypoints):
        closest_stopline = None
        closest_coordinates = None
        min_distance = np.inf

        near_waypoints_kdtree = KDTree(near_waypoints)

        for stopline_id, stopline_node in stopline.items():
            coordinates = stopline_node[0]
            stp_mid_point = [(c1 + c2) / 2 for c1, c2 in zip(*coordinates)]
            
            # 인접 waypoints가 정지선을 포함하는지 확인
            per_distance, _ = near_waypoints_kdtree.query(stp_mid_point)
            
            if per_distance > 2: 
                continue

            distance = (car_position[0] - stp_mid_point[0])**2 + (car_position[1] - stp_mid_point[1])**2
            if distance < min_distance:
                min_distance = distance
                closest_stopline = stopline_id
                closest_coordinates = coordinates

        along_track_error = 0

        if closest_stopline is None:
            along_track_error = 1000000
            return along_track_error, closest_stopline
        
        point1 = closest_coordinates[0]
        point2 = closest_coordinates[1]

        dx = point2[0] - point1[0]
        dy = point2[1] - point1[1]
        rad = np.arctan2(dy, dx) # atan2는 두 수의 비율에 대한 아크탄젠트를 반환
        
        m = dy/dx # 기울기
        b = point1[1] - m * point1[0] # y절편

        # 점과 직선의 거리
        along_track_error = abs(m * car_position[0] - car_position[1] + b) / np.sqrt(m ** 2 + 1)

        return along_track_error, closest_stopline
     
if __name__ == "__main__":
    try:
        # ROS
        localization = Localization()
        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass