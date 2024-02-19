#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from sklearn.linear_model import LinearRegression
from scipy.spatial import KDTree

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, PoseArray, Pose, PoseStamped, Point
import tf.transformations as tf_trans

from osm_handler import OSMHandler

class GlobalPathPlanning():
    def __init__(self, osm_file_path, osm_file_list):
        # OSM Handler
        osmhandler = OSMHandler()

        # import the osm files
        for osm_file in osm_file_list:
            osmhandler.import_file(osm_file_path + '/' + osm_file)

        # Attributes for ways and nodes
        self.ways = osmhandler.ways
        self.way_nodes = osmhandler.way_nodes
        self.ways_kdtree = self.kdtree(self.ways) # { way_id : points(kdtree) }

        self.mission = osmhandler.mission_areas
        self.mission_nodes = osmhandler.mission_nodes
        self.mission_types = osmhandler.mission_types

        # Attributes for global path planning
        self.clicked_point_list = []
        self.selected_ways = []
        self.selected_ways_list = []
        self.cur_way = None

    def find_candidate_ways(self, cur_way_id):
        candidate_ways = []
        cur_way = self.ways[cur_way_id]
        
        for way_id in self.ways:
            if (way_id == cur_way_id):
                continue
            
            # 첫 번째 조건: 현재 way의 마지막 노드와 특정 way의 처음 노드가 일정 거리 이내일 때
            way = self.ways[way_id]

            next_way_start_node = self.way_nodes[way[0]]
            cur_way_end_node = self.way_nodes[cur_way[-1]]
            dist = np.sqrt((next_way_start_node[0]-cur_way_end_node[0])**2 + (next_way_start_node[1]-cur_way_end_node[1])**2)

            if dist > 10:
                continue

            # 두 번째 조건: 현재 way의 마지막 부분 기울기와 특정 way의 처음 부분 기울기가 같은 부호일 때 or 비율로 ~% 이상일 때
            cur_way_direction = np.rad2deg(np.arctan2(self.way_nodes[cur_way[-1]][1]-self.way_nodes[cur_way[-2]][1], self.way_nodes[cur_way[-1]][0]-self.way_nodes[cur_way[-2]][0]))
            next_way_direction = np.rad2deg(np.arctan2(self.way_nodes[way[1]][1]-self.way_nodes[way[0]][1], self.way_nodes[way[1]][0]-self.way_nodes[way[0]][0]))

            direction_similarity = abs((cur_way_direction - next_way_direction))
            if direction_similarity < 30:
                candidate_ways.append(way_id)

        return candidate_ways
    
    def find_closest_way(self, cur_position):
        """ 현재 위치로부터 가장 가까운 way 찾기 """
        closest_way = None
        min_distance = np.inf

        for way_id, tree in self.ways_kdtree.items():
            distance, index = tree.query(cur_position)
            
            if distance < min_distance:
                min_distance = distance
                closest_way = way_id

        # """ 현재 위치로부터 가장 가까운 way 찾기 """
        # closest_way = None
        # min_distance = np.inf

        # for way_id in self.ways:
        #     for node_id in self.ways[way_id]:
        #         node_coord = self.way_nodes[node_id]
        #         distance = self.euclidean_distance(cur_position, node_coord)
        #         if distance < min_distance:
        #             min_distance = distance
        #             closest_way = way_id

        return closest_way
    
    def update_current_way(self, position):
        self.cur_way = self.selected_ways[0]
        if self.cur_way == self.selected_ways[-1]:
            return self.selected_ways[-1]
        
        next_way = self.ways[self.selected_ways[1]]
        next_way_start_node = self.way_nodes[next_way[0]]
        next_dist = self.euclidean_distance(position, next_way_start_node)
        if next_dist < 3:
            self.selected_ways.pop(0) # 맨 앞 인덱스 값 제거
            self.cur_way = self.selected_ways[0]
        
        return self.cur_way
    
    def update_selected_ways(self):
        target_ways = []
        target_point = self.clicked_point_list[-1]

        if len(self.clicked_point_list) == 1:
            first_way = self.find_closest_way(self.clicked_point_list[0])
            target_ways.append(first_way)

        else:    
            start_way = self.find_closest_way(self.clicked_point_list[-2])
            last_way = self.find_closest_way(self.clicked_point_list[-1])
        
            i = 0
            cur_way = start_way
            while True:
                if(cur_way == last_way):
                    break

                candidate_ways = self.find_candidate_ways(cur_way)
                cur_way = self.choose_candidate_way(candidate_ways, target_point)
                target_ways.append(cur_way)

                i += 1
                if(i>7):
                    break
        
        self.selected_ways += target_ways
        self.selected_ways_list.append(target_ways)
        
        return target_ways

    def choose_candidate_way(self, candidate_ways, target_point):
        cost_prev = np.inf
        next_way = None

        for candidate_way_id in candidate_ways:
            candidate_way = self.ways[candidate_way_id]
            cost = (self.way_nodes[candidate_way[-1]][0]-target_point[0])**2 + (self.way_nodes[candidate_way[-1]][1]-target_point[1])**2

            if (cost < cost_prev):
                next_way = candidate_way_id
                cost_prev = cost

        return next_way
    
    def reset_selected_ways(self):
        self.clicked_point_list = []
        self.selected_ways = []
        return
    
    def remove_target_ways(self):
        if len(self.clicked_point_list) == 0:
            return
        
        for way in self.selected_ways_list[-1]:
            self.selected_ways.remove(way)
        self.selected_ways_list.pop()
        self.clicked_point_list.pop()

        return
    
    def update_clicked_point(self, clicked_point):
        self.clicked_point_list.append(clicked_point)
        return
    
    def calculate_global_ct_error(self, closest_node_id, position):
        try:
            first_node = self.way_nodes[closest_node_id]
            second_node = self.way_nodes[closest_node_id+1]
            # 계산 로직 계속...
        
        except IndexError:
            first_node = self.way_nodes[closest_node_id -1]
            second_node = self.way_nodes[closest_node_id]
        
        slope = (second_node[1] - first_node[1])/(second_node[0] - first_node[0])
        intercept = first_node[1] - slope*first_node[0]

        if intercept >= 0:
            line_coef = [slope, -1, intercept] # ax-y+b=0
        # elif waypoint 방향 감소: -ax +y -b=0
        else:
            line_coef = [slope, 1, intercept] # ax+y+b=0
        
        cross_track_error = (line_coef[0]*position[0] + line_coef[1]*position[1] + line_coef[2])\
                                / np.sqrt(line_coef[0]**2 + line_coef[1]**2)
        
        return cross_track_error
    
    # def calculate_global_ct_error(self, closest_nodes, position):
    #     xs, ys = closest_nodes[:,0], closest_nodes[:,1]

    #     model = LinearRegression() # 선형 회귀 모델 적용
    #     X = xs.reshape(-1,1)
    #     y = ys.reshape(-1,1)
    #     model.fit(X, y)

    #     linear_vector = closest_nodes[1] - closest_nodes[0]
    
    #     # if waypoint 방향 증가: ax-y+b=0
    #     if linear_vector[0] >= 0:
    #         line_coef = [model.coef_[0][0], -1, model.intercept_[0]] # ax-y+b=0
    #     # elif waypoint 방향 감소: -ax +y -b=0
    #     elif linear_vector[0] < 0:
    #         line_coef = [-model.coef_[0][0], 1, -model.intercept_[0]] # ax-y+b=0
        
    #     cross_track_error = (line_coef[0]*position[0] + line_coef[1]*position[1] + line_coef[2])\
    #                             / np.sqrt(line_coef[0]**2 + line_coef[1]**2)
        
    #     return cross_track_error
    
    def find_closest_nodes(self, cur_way, position):
        _, indices = self.ways_kdtree[cur_way].query(position, 7)
        closest_nodes_id = [self.ways[cur_way][i] for i in indices]
        closest_node_id = closest_nodes_id[0]
        closest_nodes_id.sort()
        
        closest_nodes = np.array([self.way_nodes[node_id] for node_id in closest_nodes_id])
        return closest_nodes, closest_nodes_id, closest_node_id
    
    def make_way_msg(self, ways_for_visualize):
        ways = PoseArray()
        ways.header.frame_id = "utm"

        for way_id in ways_for_visualize:
            position_prev = None
            for node_id in self.ways[way_id]:
                position = self.way_nodes[node_id]

                # 첫 번째 점이면 continue
                if position_prev is None:
                    position_prev = position
                    continue
                
                # 방향 계산
                yaw = np.arctan2(position[1] - position_prev[1], position[0] - position_prev[0])
                quaternion = tf_trans.quaternion_from_euler(0, 0, yaw)

                # Pose 생성
                pose = Pose()
                pose.position.x, pose.position.y = position
                pose.orientation.x = quaternion[0]
                pose.orientation.y = quaternion[1]
                pose.orientation.z = quaternion[2]
                pose.orientation.w = quaternion[3]
                ways.poses.append(pose)
                position_prev = position # 현재 위치 정보 업데이트
        
        return ways
    
    def make_mission_msg(self, areas_for_visualize):
        mission = MarkerArray()

        for area_id in areas_for_visualize:
            polygon = Marker()
            polygon.header.frame_id = 'utm'
            polygon.type = Marker.LINE_STRIP
            polygon.action = Marker.ADD
            polygon.id = area_id

            # Marker scale
            polygon.scale.x = 1.0
            polygon.scale.y = 1.0

            # Marker color
            polygon.color.a = 1.0
            polygon.color.r = 0.0
            polygon.color.g = 1.0
            polygon.color.b = 1.0

            x_centers, y_centers = [], []
            for node_id in self.mission[area_id]:
                position = self.mission_nodes[node_id]
                p = Point()
                p.x = position[0]
                p.y = position[1]
                polygon.points.append(p)

                x_centers.append(p.x)
                y_centers.append(p.y)

            mission.markers.append(polygon)

            x_center = np.array(x_centers[:-1]).sum() / (len(self.mission[area_id])-1)
            y_center = np.array(y_centers[:-1]).sum() / (len(self.mission[area_id])-1)
            
            text_marker = Marker()
            text_marker.header.frame_id = 'utm'
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.id = area_id + 10000

            text_marker.pose.position.x = x_center  # x 위치
            text_marker.pose.position.y = y_center  # y 위치
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.3 # text scale, mapviz에서 적용이 안됨

            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 0.0

            text_marker.text = self.mission_types[area_id]
            mission.markers.append(text_marker)

        return mission
    
    def kdtree(self, ways):
        ways_kdtree = {}
        for way_id in ways:
            waypoints = [self.way_nodes[node_id] for node_id in self.ways[way_id]] # [(x1, y1), (x2, y2), ...]
            ways_kdtree[way_id] = KDTree(waypoints)
            
        return ways_kdtree
    
    @staticmethod
    def euclidean_distance(pos1, pos2):
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    