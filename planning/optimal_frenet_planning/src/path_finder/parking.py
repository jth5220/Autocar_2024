#! /usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np
from scipy.spatial import KDTree
import math
import time

def rotate_point(x, y, yaw):
    # 회전 변환 행렬 적용
    x_new = x * math.cos(yaw) - y * math.sin(yaw)
    y_new = x * math.sin(yaw) + y * math.cos(yaw)
    return x_new, y_new

def points_on_circle(center, radius, start_angle, end_angle, num_points=15):
    angles = np.linspace(start_angle, end_angle, num_points)
    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    return x, y

def create_straight_path(start, end, step_size):

    x_start, y_start = start
    x_end, y_end = end
    
    # Calculate the total distance
    total_distance = np.sqrt((x_end - x_start)**2 + (y_end - y_start)**2)
    
    # Calculate the number of points
    num_points = int(total_distance / step_size) + 1
    
    # Generate points along the path
    x_path = np.linspace(x_start, x_end, num_points)
    y_path = np.linspace(y_start, y_end, num_points)
    
    return x_path, y_path

class Parking():
    def __init__(self, ref_path, car_pose_init, min_R):
        self.ref_path = ref_path
        self.car_pose_init = car_pose_init

        self.min_R = min_R # 최소 회전 반경 [m]
        
        self.detect_parking_spot = False
        self.parking_status = -1
        # parking_status 
        # 0: staright
        # 1: backward circle
        # 2: forward circle
        # 3: ref path 따라가기
        self.parking_stop_time = None

        self.parking_path_straight_x, self.parking_path_straight_y = None, None
        self.parking_path_circle_x, self.parking_path_circle_y = None, None

        self.ref_path_kdtree = KDTree(list(zip(ref_path['x'], ref_path['y'])))

        return
    
    def find_path(self, car_pose, obstacles):
        # 주차 스팟 찾기
        if not self.detect_parking_spot:
            # utm -> base_link
            obstacles_bl = self.transform_obstacles_utm_2_bl(car_pose, obstacles)

            # ROI-y축 범위에 대해서 filtering (차 기준 오른쪽 라바콘만 잡히게)
            obstacles_filtered = self.filter_obstacles(obstacles_bl)

            if len(obstacles_filtered) <= 2:
                path_x, path_y = self.find_ref_path(car_pose)
                return [], [path_x, path_y, self.calc_path_yaw(path_x, path_y)], False, 0
            
            # x값 작은 순서대로 분류 (x값 기준 차에서 가까운 순서대로)
            obstacles_sorted = obstacles_filtered[np.argsort(obstacles_filtered[:, 0])] 
            distances = np.sqrt(np.diff(obstacles_sorted[:, 0])**2 + np.diff(obstacles_sorted[:, 1])**2)

            is_found = self.find_parking_path(car_pose, obstacles_sorted, distances)

            if is_found:
                path_x, path_y = self.parking_path_straight_x, self.parking_path_straight_y
                self.detect_parking_spot = True
                self.parking_status = 0

            elif not is_found:
                path_x, path_y = self.find_ref_path(car_pose)
            
            gear = 0

        elif self.detect_parking_spot:

            if self.parking_status == 0:
                path_x, path_y = self.parking_path_straight_x, self.parking_path_straight_y

            elif self.parking_status == 1:
                path_x, path_y = self.parking_path_circle_x, self.parking_path_circle_y

            elif self.parking_status == 2:
                path_x, path_y = self.parking_path_circle_x[::-1], self.parking_path_circle_y[::-1]
                
            elif self.parking_status >= 3:
                path_x, path_y = self.find_ref_path(car_pose)

            # 현재 path에서 끝부분에 도착하면 다음 status로 변경
            path_kdtree = KDTree(list(zip(path_x, path_y)))
            _, cur_idx = path_kdtree.query(car_pose[:2])
            if cur_idx >= len(path_x)-3:
                self.parking_status += 1
                self.parking_stop_time = time.time()

            print("parking_status: ", self.parking_status)

            gear = self.get_gear_parking()

        return [], [path_x, path_y, self.calc_path_yaw(path_x, path_y)], False, gear

    def get_gear_parking(self):
        if self.parking_status == 0:
            gear = 0

        elif self.parking_status == 1:
            if time.time() - self.parking_stop_time <= 2: # neutral
                gear = 1
            else: gear = 2

        elif self.parking_status == 2:
            if time.time() - self.parking_stop_time <= 10: # neutral
                gear = 1
            else: gear = 0

        elif self.parking_status >= 3:
            gear = 0

        return gear
    
    def find_parking_path(self, car_pose, obstacles, distances):
        is_found = False

        for i, dist in enumerate(distances):
            if dist < 4:
                continue
            
            is_found = True
            target_idx = i
        
        if is_found:
            # 주차위치
            parking_spot = (obstacles[target_idx] + obstacles[target_idx+1]) / 2
            parking_spot[0] -= 1.0 #세로
            parking_spot[1] -= 1.0 #가로

            print("주차 스팟: ",parking_spot)

            dx = obstacles[target_idx+1][0] - obstacles[target_idx][0]
            dy = obstacles[target_idx+1][1] - obstacles[target_idx][1]
            parking_yaw = np.arctan2(dy, dx)

            parking_spot[0], parking_spot[1] = rotate_point(parking_spot[0], parking_spot[1], -parking_yaw)

            # 원2의 중심과 반지름
            center2 = (parking_spot[0], parking_spot[1] + self.min_R)
            radius2 = self.min_R

            # 원1의 중심과 반지름
            y1 = 0 - self.min_R
            distance = self.min_R * 2
            delta_y = center2[1] - y1
            delta_x = np.sqrt(distance**2 - delta_y**2)
            x1 = parking_spot[0] + delta_x
            center1 = (x1, y1)
            radius1 = self.min_R

            # dis = np.sqrt((x1-parking_spot[0])**2+(center2[1]-y1))
            p0 = (-1.0,0)
            p1 = (center1[0],0)
            p2 = ((center1[0]+center2[0])/2, (center1[1]+center2[1])/2)
            p3 = (parking_spot[0], parking_spot[1])

            # path1(직진) (현재위치 - p1)
            step_size = 0.2
            rotate_path1_x, rotate_path1_y = create_straight_path(p0, p1, step_size)
            path1_x = []
            path1_y = []
            
            for x, y in zip(rotate_path1_x, rotate_path1_y):
                x_new, y_new = rotate_point(x, y, parking_yaw)        
                path1_x.append(x_new)
                path1_y.append(y_new)
            
            path1_x_utm, path1_y_utm = self.transform_waypoints_bl_2_utm(car_pose, path1_x, path1_y)

            # path2
            # 시작점에서 첫 번째 원의 둘레를 따라 점 생성(점 15개)
            start_angle1 = np.arctan2(p1[1] - center1[1], p1[0] - center1[0])
            end_angle1 = np.arctan2(p2[1] - center1[1], p2[0] - center1[0])
            p_x1, p_y1 = points_on_circle(center1, radius1, start_angle1, end_angle1)

            # 두 번째 원의 둘레를 따라 점 생성(점 15개)
            start_angle2 = np.arctan2(p2[1] - center2[1], p2[0] - center2[0])
            end_angle2 = np.arctan2(p3[1] - center2[1], p3[0] - center2[0])
            p_x2, p_y2 = points_on_circle(center2, radius2, start_angle2, end_angle2)

            rotate_path2_x = np.concatenate([p_x1, p_x2])
            rotate_path2_y = np.concatenate([p_y1, p_y2])
            path2_x = []
            path2_y = []
            for x, y in zip(rotate_path2_x, rotate_path2_y):
                x_new, y_new = rotate_point(x, y, parking_yaw)        
                path2_x.append(x_new)
                path2_y.append(y_new)

            path2_x_utm, path2_y_utm = self.transform_waypoints_bl_2_utm(car_pose, path2_x, path2_y)

            self.parking_path_straight_x, self.parking_path_straight_y = path1_x_utm, path1_y_utm
            self.parking_path_circle_x, self.parking_path_circle_y = path2_x_utm, path2_y_utm

            return True

        else:
            return False
    
    def find_ref_path(self, car_pose):
        _, idx = self.ref_path_kdtree.query(car_pose[:2])

        n_back = 4
        n_forward = 15
        start_index = max(idx - n_back, 0)
        end_index = min(idx + n_forward, len(self.ref_path['x']))
        
        # 인근 waypoints 추출
        path_x = self.ref_path['x'][start_index:end_index + 1]
        path_y = self.ref_path['y'][start_index:end_index + 1]
        return path_x, path_y
    
    @staticmethod
    def transform_waypoints_bl_2_utm(car_pose, path_x_bl, path_y_bl):
        path_x_global = []
        path_y_global = []

        # Convert each point in the path to the global frame
        for x_bl, y_bl in zip(path_x_bl, path_y_bl):
            # Rotate the points
            x_rot = x_bl * math.cos(car_pose[2]) - y_bl * math.sin(car_pose[2])
            y_rot = x_bl * math.sin(car_pose[2]) + y_bl * math.cos(car_pose[2])

            # Translate the points
            x_global = x_rot + car_pose[0]
            y_global = y_rot + car_pose[1]

            # Append the transformed coordinates to the lists
            path_x_global.append(x_global)
            path_y_global.append(y_global)

        return path_x_global, path_y_global
    
    @ staticmethod
    def calc_path_yaw(path_x, path_y):
        path_yaw = []
        for i in range(len(path_x) - 2):
            # Calculate the differences in x and y
            dx = path_x[i + 1] - path_x[i]
            dy = path_y[i + 1] - path_y[i]

            # Calculate the yaw angle using atan2, which handles all quadrants
            yaw = math.atan2(dy, dx)

            # Append the calculated yaw to the list
            path_yaw.append(yaw)
        
        path_yaw.append(path_yaw[-1])
        return path_yaw

    def bl_2_utm(self, car_pose, point):
        x_rot = point[0] * math.cos(car_pose[2]) - point[1] * math.sin(car_pose[2])
        y_rot = point[0] * math.sin(car_pose[2]) + point[1] * math.cos(car_pose[2])

        # Translate the points
        x_global = x_rot + car_pose[0]
        y_global = y_rot + car_pose[1]
        return (x_global, y_global)
    
    def utm_2_bl(self, car_pose, point):
        # Translate the obstacle coordinates
        translated_x = point[0] - car_pose[0]
        translated_y = point[1] - car_pose[1]

        # Rotate the coordinates to align with the car's orientation
        base_link_x = translated_x * math.cos(-car_pose[2]) - translated_y * math.sin(-car_pose[2])
        base_link_y = translated_x * math.sin(-car_pose[2]) + translated_y * math.cos(-car_pose[2])
        return (base_link_x, base_link_y)
    
    @staticmethod
    def transform_obstacles_utm_2_bl(car_pose, obstacles):
        transformed_obstacles = []

        for obstacle in obstacles:
            obs_x, obs_y, r = obstacle

            # Translate the obstacle coordinates
            translated_x = obs_x - car_pose[0]
            translated_y = obs_y - car_pose[1]

            # Rotate the coordinates to align with the car's orientation
            base_link_x = translated_x * math.cos(-car_pose[2]) - translated_y * math.sin(-car_pose[2])
            base_link_y = translated_x * math.sin(-car_pose[2]) + translated_y * math.cos(-car_pose[2])

            # Keep the radius the same
            transformed_obstacles.append([base_link_x, base_link_y, r])

        return transformed_obstacles
    
    @staticmethod
    def filter_obstacles(obstacles):
        obstacles_filtered = []
        for ob in obstacles:
            if -3.0 <= ob[1] <= -1.0:
                obstacles_filtered.append(ob)

        obstacles_filtered = np.array(obstacles_filtered)

        return obstacles_filtered