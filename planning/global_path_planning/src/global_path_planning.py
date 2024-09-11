#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import numpy as np
from scipy.spatial import KDTree

from osm_handler import OSMHandler
from keyboard_input import KeyboardInput
from way_selector import WaySelector

import rospy
import rospkg
from geometry_msgs.msg import PointStamped, PoseArray, Pose, Point
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32, Bool, Int8
from tf.transformations import quaternion_from_euler

# ==================================================================================
import pyproj
def latlon_to_utm(lat, lon):
    proj = '+proj=utm +zone=52 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
    latlon_to_utm = pyproj.Proj(proj, preserve_units=True)
    return latlon_to_utm(lon, lat)

def euclidean_distance(pos1, pos2):
    return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
# ==================================================================================

msg = """
======= Global path planning =========

Left mouse click: 경로 선택

q: 최근에 선택한 경로 취소

w: 선택했던 경로 다시 Publish

Backspace: 모든 선택한 경로 취소

======================================
"""
rospack = rospkg.RosPack()
OSM_FILE_PATH = rospack.get_path('global_path_planning') + "/osm_files"
# OSM_FILE_LIST = ["hitech_LINK.osm", "hitech_INTERSECTION_LINK.osm", "hitech2_STOPLINE.osm"]
# OSM_FILE_LIST = ["KCITY_LINK_rev.osm", "KCITY_INTERSECTION_LINK_rev.osm"]
OSM_FILE_LIST = ['KCITY_INTERSECTION_LINK_MAIN.osm', 'KCITY_LINK_MAIN.osm', 'KCITY_STOPLINE_0910.osm',
                 'kcity_MISSION.osm']

class GlobalPathPlanning():
    def __init__(self):
        self.__set_attributes__()
        self.__set_ros__()
        print(msg)
        print("======== 불러온 목록 ==========")
        print(f"주행유도선 노드: {len(self.way_nodes)}개")
        print(f"미션 구역: {len(self.mission)}개")
        print(f"정지선: {len(self.stopline)}개")
        print("============================")

        return
    
    def __set_attributes__(self):
        self.is_published_once = False
        self.selected_ways = None
        self.location_corrected = None

        # Driving에서 쓰이는 변수들
        self.waypoints = []
        self.waypoints_id = []
        self.cur_way_idx = 0
        self.prev_way = None
        self.possible_change_direction = 'both'

        # traffic light
        self.traffic_detected = 'None'
        self.is_traffic_detecting = False
        self.is_traffic_finished = False
        self.gear_override = 0 # 0: Forward / 1: Neutral

        self.driving_mode = 'normal_driving' # ['normal_driving', 'obstacle_avoiding', 'intersect', 'parking', 'delivery']
        self.is_avoiding = False

        # OSM Handler
        osmhandler = OSMHandler()

        # import the osm files
        for osm_file in OSM_FILE_LIST:
            osmhandler.import_file(OSM_FILE_PATH + '/' + osm_file)

        # Attributes for ways and nodes
        self.ways = osmhandler.ways # {way1: [node1, node2, ...], way2:[node11,node12,...]}
        self.way_nodes = osmhandler.way_nodes # {node1:[x1,y1], node2:[x2,y2], ... }
        self.ways_info = osmhandler.ways_info
        self.mission_way = {}

        self.mission = osmhandler.mission_areas
        self.mission_nodes = osmhandler.mission_nodes
        self.mission_types = osmhandler.mission_types

        self.stopline = osmhandler.stopline
        self.stopline_nodes = osmhandler.stopline_nodes
        self.stopline_way = {}

        # WaySelector
        self.way_selector = WaySelector(self.ways, self.way_nodes)

        # Key Input
        self.keyboard_input = KeyboardInput(rate=0.05)
        return
    
    def __set_ros__(self):
        # ROS
        rospy.init_node('global_path_planning')
        
        # Subscribers
        self.clicked_point_sub = rospy.Subscriber('/clicked_point', PointStamped, self.callback_selecting_ways_by_clicking)
        self.location_corrected_sub = rospy.Subscriber('/location_corrected', Odometry, self.callback_corrected_location) # 종방향 에러 계산할 때
        self.location_long_corrected_sub = rospy.Subscriber('/location_long_corrected', Odometry, self.callback_long_corrected_location) # ct error 계산할 때
        self.is_avoiding_sub = rospy.Subscriber('/is_avoiding', Bool, self.callback_is_lane_changing)
        self.traffic_sub = rospy.Subscriber('/traffic_sign', String, self.callback_traffic)

        # Publishers
        # =================== Way selector =======================
        self.way_pub = rospy.Publisher('/ways', PoseArray, queue_size=1) # 모든 ways
        
        self.mission_areas_pub = rospy.Publisher('/mission_areas', MarkerArray, queue_size=1) # 미션구역
        self.stoplines_pub = rospy.Publisher('/stoplines', MarkerArray, queue_size=1) # 미션구역
        
        self.clicked_way_pub = rospy.Publisher('/clicked_closest_way', PoseArray, queue_size=1) # 해당 지점에서 제일 가까운 way
        self.candidate_ways_pub = rospy.Publisher('/candidates_ways', PoseArray, queue_size=1) # 제일 가까운 way 다음의 후보 ways
        self.selected_ways_pub = rospy.Publisher('/selected_ways', PoseArray, queue_size=1) # target ways
        
        # =================== 주행 중 =======================
        self.closest_waypoints_pub = rospy.Publisher('/global_closest_waypoints', PoseArray, queue_size=10)
        self.waypoints_pub = rospy.Publisher('/global_waypoints', PoseArray, queue_size=10)

        # cte, ate 묶어서 publish 수정 => publish 토픽 수 줄이기 가능
        self.cte_global_pub = rospy.Publisher('/ct_error_global', Float32, queue_size=10)
        self.ate_global_pub = rospy.Publisher('/at_error_global', Float32, queue_size=10) # 횡방향 보정위치부터 가장 가까운 정지선 거리
        
        self.driving_mode_pub = rospy.Publisher('/driving_mode', String, queue_size=10)

        self.traffic_mode_pub = rospy.Publisher('/mode/traffic', Bool, queue_size=10)
        self.gear_override_pub = rospy.Publisher('/gear/override', Int8, queue_size=10)

        self.timer_driving = rospy.Timer(rospy.Duration(0.1), self.callback_timer_driving)
        self.timer_selecting = rospy.Timer(rospy.Duration(0.1), self.callback_timer_selecting_ways_by_key_input)
        return
    
    def callback_timer_driving(self, event):
        if self.is_published_once == False:
            if self.way_pub.get_num_connections() == 1:
                self.publish_all_objects()
                self.is_published_once = True
            return
        
        # 아직 주행 준비 X => Early return
        if self.location_corrected is None or len(self.way_selector.selected_ways) == 0:
            return
        
        first_time = time.time()
        # 가장 가까운 waypoint 노드 찾기 / idx와 id 잘 구별하기
        # idx: list 상에서 인덱스 번호
        # id: 처음에 global path objects 불러왔을 때의 노드 id

        # cur way update
        cur_way = self.update_current_way(self.location_corrected)
        # print(self.way_selector.selected_ways, "중에서 현재 way: ", cur_way, '\n')

        self.driving_mode = self.update_driving_mode(cur_way)

        # 차선 변경 가능한 방향
        possible_change_direction = self.update_change_direction(cur_way)
        
        # waypoints
        waypoints, near_waypoints, closest_wp_idx = self.get_waypoints()
        
        # cross track error
        cte = self.get_cte(self.location_corrected, waypoints, closest_wp_idx)
        # print("cte: ", cte)
        
        # along track error
        ate = self.get_ate(cur_way, self.location_corrected)
        # print("ate:", ate)
        # print("traffic:", self.ways_info['traffic'][cur_way])

        # 신호등 체크
        self.update_traffic(ate, cur_way) 
        # print('gear_override:', gear_override)

        # ROS
        # near_waypoints publish (For visualization)
        # near_waypoints_msg = self.make_waypoints_msg(near_waypoints, 'utm')
        # self.closest_waypoints_pub.publish(near_waypoints_msg)

        # waypoints & possible_change_direction publish
        if cur_way != self.prev_way: # cur_way가 바뀌면
            # 근처 ways에 대한 waypoints publish
            # 차선 변경한 방향도 같이 publish (frame_id에 넣어서)
            global_waypoints_msg = self.make_waypoints_msg(waypoints, possible_change_direction)
            self.waypoints_pub.publish(global_waypoints_msg)
            self.prev_way = cur_way

            self.is_traffic_finished = False
        
        # global cte publish
        cte_msg = Float32()
        cte_msg.data = cte
        self.cte_global_pub.publish(cte_msg)

        # global ate publish
        ate_msg = Float32()
        ate_msg.data = ate
        self.ate_global_pub.publish(ate_msg)

        # driving mode publish
        driving_mode_msg = String()
        driving_mode_msg.data = self.driving_mode
        self.driving_mode_pub.publish(driving_mode_msg)

        # gear override publish
        gear_override_msg = Int8()
        gear_override_msg.data = self.gear_override
        self.gear_override_pub.publish(gear_override_msg)

        # traffic mode publish
        traffic_mode_msg = Bool()
        traffic_mode_msg.data = self.is_traffic_detecting
        self.traffic_mode_pub.publish(traffic_mode_msg)

        # print("소요 시간: ", time.time() - first_time)
        return
    
    def callback_corrected_location(self, location_msg):
        self.location_corrected = (location_msg.pose.pose.position.x, location_msg.pose.pose.position.y)
        return
    
    def callback_long_corrected_location(self, location_msg):
        self.location_long_corrected = (location_msg.pose.pose.position.x, location_msg.pose.pose.position.y)
        return
    
    def callback_is_lane_changing(self, is_avoiding_msg):
        self.is_avoiding = is_avoiding_msg.data
        return
    
    def callback_traffic(self, traffic_msg):
        self.traffic_detected = [item.strip() for item in traffic_msg.data.split(',')]  
        return
    
    def update_mission_status(self, cur_way, car_pos):
        cur_mission_id = self.mission_way.get(cur_way, None)

        if cur_mission_id is None:
            return None, False
        
        mission_type = self.mission_types[cur_mission_id]
        mission_node_ids = self.mission[cur_mission_id]
        
        mission_nodes = [self.mission_nodes[node_id] for node_id in mission_node_ids[:4]]

        is_inside = self.is_point_in_rectangle(mission_nodes, car_pos)

        return mission_type, is_inside
    
    def is_point_in_rectangle(self, rect_points, point):
        def cross_product(o, a, b):
            # 벡터 OA와 OB의 크로스 프로덕트를 계산합니다
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
        
        # 네 점을 각각 A, B, C, D로 정의합니다
        A, B, C, D = rect_points
        
        # 점이 사각형의 각 변에 대해 같은 방향에 있는지를 확인합니다
        # 크로스 프로덕트가 양수 또는 음수인지를 통해 점의 위치를 파악합니다
        # 시계 방향 기준으로 모든 외적이 양수이거나 음수여야 내부에 있는 것입니다
        cp1 = cross_product(A, B, point)
        cp2 = cross_product(B, C, point)
        cp3 = cross_product(C, D, point)
        cp4 = cross_product(D, A, point)
        
        # 모두 같은 부호면 내부에 있는 것입니다
        return (cp1 >= 0 and cp2 >= 0 and cp3 >= 0 and cp4 >= 0) or (cp1 <= 0 and cp2 <= 0 and cp3 <= 0 and cp4 <= 0)
        
    def update_driving_mode(self, cur_way):
        driving_mode = None

        # cur way에 대해서 미션 구역인지 판단 & 미션 구역 내에 들어왔는지 판단
        mission_type, is_inside_mission = self.update_mission_status(cur_way, self.location_corrected)
        print('current mission type:', mission_type, '\n')
        if is_inside_mission == True:
            driving_mode = mission_type

        elif self.is_avoiding == True:
            driving_mode = 'obstacle_avoiding'
        
        elif self.ways_info['type'][cur_way] == 1:
            driving_mode = 'intersect'

        else:
            driving_mode = 'normal_driving'

        return driving_mode
        
    def update_current_way(self, position):
        cur_way = self.way_selector.selected_ways[self.cur_way_idx]

        # 마지막 노드일 때
        if cur_way == self.way_selector.selected_ways[-1]:
            return cur_way
        
        next_way = self.ways[self.way_selector.selected_ways[self.cur_way_idx+1]]
        next_way_start_node = self.way_nodes[next_way[0]]

        # cur_way_end_node = self.way_nodes[self.ways[cur_way][-1]]

        next_dist = euclidean_distance(position, next_way_start_node)
        # cur_dist = euclidean_distance(position, cur_way_end_node)

        if next_dist < 3: # 다음 way 첫 노드로부터 3m 이내에 들어오면 다음 way로 넘어감
            self.cur_way_idx += 1
            self.prev_way = cur_way

        return cur_way
    
    def update_change_direction(self, cur_way):
        change_direction = None
        change_direction_index = self.ways_info['change_direction'][cur_way]

        if change_direction_index == 0:
            change_direction = 'both'

        elif change_direction_index == 1:
            change_direction = 'left'

        elif change_direction_index == 2:
            change_direction = 'right'

        else:
            change_direction = 'none'
        return change_direction
    
    def update_traffic(self, distance_stopline, cur_way):
        traffic_target = self.ways_info['traffic'][cur_way]
        if distance_stopline < 15 and traffic_target != 'no_traffic':
            self.is_traffic_detecting = True
        else: 
            self.is_traffic_detecting = False

        if distance_stopline < 6.0 and self.is_traffic_finished == False:
            print('traffic_detected:', self.traffic_detected)
            if traffic_target == 'left':
                if 'Left' in self.traffic_detected:
                    self.is_traffic_finished = True
                    self.gear_override = 0
                else:
                    self.gear_override = 1

            elif traffic_target == 'right':
                # self.is_traffic_finished = True
                print(self.traffic_detected)

            elif traffic_target == 'straight':
                if 'Green' in self.traffic_detected or \
                    'Straightleft' in self.traffic_detected or \
                    'Yellow' in self.traffic_detected:
                    self.is_traffic_finished = True
                    self.gear_override = 0
                    print('go')
                else:
                    self.gear_override = 1
                    print('stop')
    
    def get_waypoints(self):
        # 1. near_way 찾기
        start_index = max(self.cur_way_idx - 1, 0) # 시작 인덱스와 끝 인덱스 계산
        end_index = min(self.cur_way_idx + 1, len(self.way_selector.selected_ways))
        near_ways = self.way_selector.selected_ways[start_index:end_index + 1] # 인근한 3개의 way 추출

        # 2. 인근 ways로부터 waypoints 추출
        waypoints, waypoints_node_id = [], []
        for way_id in near_ways:
            waypoints += [self.way_nodes[node_id] for node_id in self.ways[way_id]]
            waypoints_node_id += [node_id for node_id in self.ways[way_id]]
        
        waypoints_kdtree = KDTree(waypoints)
        _, closest_node_idx = waypoints_kdtree.query(self.location_corrected)

        # 가장 가까운 노드로부터 앞으로 7개, 뒤로 4개 찾기
        n_back = 4
        n_forward = 15
        start_index = max(closest_node_idx - n_back, 0)
        end_index = min(closest_node_idx + n_forward, len(waypoints))
        
        # 인근 waypoints 추출
        near_waypoints = waypoints[start_index:end_index + 1]

        return waypoints, near_waypoints, closest_node_idx
    
    def get_cte(self, car_position, waypoints, closest_wp_idx):
        try:
            first_node = waypoints[closest_wp_idx]
            second_node = waypoints[closest_wp_idx + 1]
        except IndexError:
            first_node = waypoints[closest_wp_idx - 1]
            second_node = waypoints[closest_wp_idx]
        
        linear_vector = [(second_node[0] - first_node[0]) , (second_node[1] - first_node[1])]
        slope = linear_vector[1]/linear_vector[0]
        intercept = first_node[1] - slope*first_node[0]

        if linear_vector[0] >= 0:
            line_coef = [slope, -1, intercept] # ax-y+b=0
        # elif waypoint 방향 감소: -ax +y -b=0
        else:
            line_coef = [-slope, 1, -intercept] # ax+y+b=0
        
        cross_track_error = (line_coef[0]*car_position[0] + line_coef[1]*car_position[1] + line_coef[2])\
                                / np.sqrt(line_coef[0]**2 + line_coef[1]**2)

        return cross_track_error
    
    def get_ate(self, cur_way, car_position):
        along_track_error = 1000000

        cur_stopline_id = self.stopline_way.get(cur_way, None)
        if cur_stopline_id is None:
            return along_track_error
        
        stopline_nodes = self.stopline[cur_stopline_id]
        
        stp_points = [self.stopline_nodes[v] for v in stopline_nodes]
        stp_mid_point = [(c1 + c2) / 2 for c1, c2 in zip(*stp_points)]
        stp_dist = np.sqrt((car_position[0] - stp_mid_point[0])**2 + (car_position[1] - stp_mid_point[1])**2)

        if stp_dist > 10:
            return along_track_error
        
        point1 = stp_points[0]
        point2 = stp_points[1]

        dx = point2[0] - point1[0]
        dy = point2[1] - point1[1]
        # rad = np.arctan2(dy, dx) # atan2는 두 수의 비율에 대한 아크탄젠트를 반환
        
        m = dy/dx # 기울기
        b = point1[1] - m * point1[0] # y절편

        # 점과 직선의 거리
        along_track_error = abs(m * car_position[0] - car_position[1] + b) / np.sqrt(m ** 2 + 1)
        return along_track_error
    
    # way에 미션 구역 할당하기
    def assign_mission_way(self, selected_ways):
        # 선택한 ways에 대한 KDTree (탐색 속도 개선을 위해서)
        selected_ways_kdtree = {}
        self.mission_way = {}

        for way_id in selected_ways:
            waypoints = [self.way_nodes[node_id] for node_id in self.ways[way_id]] # [(x1, y1), (x2, y2), ...]
            selected_ways_kdtree[way_id] = KDTree(waypoints)

        # 각 미션 구역에 대
        for mission_id, misson_node_ids in self.mission.items():
            # 미션 구역 중심 찾기
            mission_cp = np.sum(np.array([self.mission_nodes[mn_id] for mn_id in misson_node_ids[:4]]), axis=0) / 4

            # 미션 구역과 제일 가까운 way 탐색
            closest_way = None
            min_distance = np.inf
            for way_id, tree in selected_ways_kdtree.items():
                distance, _ = tree.query(mission_cp)
                
                if distance > 5:
                    continue
                
                if distance < min_distance:
                    min_distance = distance
                    closest_way = way_id

            if closest_way is not None:
                self.mission_way[closest_way] = mission_id

        print('mission_way: ',self.mission_way)
        return
    
    # way에 stopline 할당하기
    def assign_stopline_way(self, selected_ways):
        # 선택한 ways에 대한 KDTree (탐색 속도 개선을 위해서)
        selected_ways_kdtree = {}
        self.stopline_way = {}

        for way_id in selected_ways:
            if self.ways_info['type'][way_id] != 0:
                continue
            waypoints = [self.way_nodes[node_id] for node_id in self.ways[way_id]] # [(x1, y1), (x2, y2), ...]
            selected_ways_kdtree[way_id] = KDTree(waypoints)
        
        # 각 정지선에 대해
        for stopline_id, stopline_node in self.stopline.items():
            coordinates = [self.stopline_nodes[v] for v in stopline_node]
            stp_mid_point = [(c1 + c2) / 2 for c1, c2 in zip(*coordinates)]

            # 미션 구역과 제일 가까운 way 탐색
            closest_way = None
            min_distance = np.inf
            for way_id, tree in selected_ways_kdtree.items():
                distance, _ = tree.query(stp_mid_point)

                if distance > 5:
                    continue

                if distance < min_distance:
                    min_distance = distance
                    closest_way = way_id
            
            if closest_way is not None:
                self.stopline_way[closest_way] = stopline_id
        
        return
    
    # ==== Way Selecting =======================================================
    def callback_selecting_ways_by_clicking(self, clicked_point_msg):
        # Mapviz 상에서 마우스 좌클릭한 위치 반환
        clicked_point = latlon_to_utm(clicked_point_msg.point.y, clicked_point_msg.point.x)

        # way_selectotor - clicked_point_list 업데이트
        clicked_way, candidate_ways, recently_selected_ways =\
                            self.way_selector.update_click_input(clicked_point)
        
        print("현재 선택된 way: ", self.way_selector.selected_ways)

        # 미션 구역 중에서 제일 가까운 way 찾기
        self.assign_mission_way(self.way_selector.selected_ways)
        self.assign_stopline_way(self.way_selector.selected_ways)

        # ROS msg 생성 및 Publish
        clicked_way_msg = self.make_way_msg([clicked_way])
        candidate_ways_msg = self.make_way_msg(candidate_ways)
        selected_ways_msg = self.make_way_msg(self.way_selector.selected_ways)

        self.clicked_way_pub.publish(clicked_way_msg)
        self.candidate_ways_pub.publish(candidate_ways_msg)
        self.selected_ways_pub.publish(selected_ways_msg)

        self.prev_way = None
        return

    def callback_timer_selecting_ways_by_key_input(self, event):
        key_input = self.keyboard_input.update()
        key_input_list = ['\x7f', 'q', 'w', 'a', '\x03']

        if key_input not in key_input_list:
            return 
        
        if key_input == '\x7f':
            self.way_selector.reset_selected_ways()
            selected_ways_msg = self.make_way_msg(self.way_selector.selected_ways)
            self.selected_ways_pub.publish(selected_ways_msg)

            clicked_way_msg = self.make_way_msg([])
            self.clicked_way_pub.publish(clicked_way_msg)

            candidate_ways_msg = self.make_way_msg([])
            self.candidate_ways_pub.publish(candidate_ways_msg)

        elif key_input == 'q':
            self.way_selector.remove_target_ways()
            selected_ways_msg = self.make_way_msg(self.way_selector.selected_ways)
            self.selected_ways_pub.publish(selected_ways_msg)

            clicked_way_msg = self.make_way_msg([])
            self.clicked_way_pub.publish(clicked_way_msg)

            candidate_ways_msg = self.make_way_msg([])
            self.candidate_ways_pub.publish(candidate_ways_msg)

        elif key_input == 'w':
            selected_ways_msg = self.make_way_msg(self.way_selector.selected_ways)
            self.selected_ways_pub.publish(selected_ways_msg)
        
        elif key_input == 'a':
            self.publish_all_objects()

        elif key_input == '\x03':
            rospy.signal_shutdown("Ctrl+C has been pressed.")
        
        print("현재 선택된 way: ", self.way_selector.selected_ways)
        return

    def publish_all_objects(self):
        print("All objects are published.")
        print("")
        all_ways = self.make_way_msg(self.ways)
        self.way_pub.publish(all_ways)

        all_mission_areas = self.make_mission_msg(self.mission)
        self.mission_areas_pub.publish(all_mission_areas)
        
        all_stoplines = self.make_stopline_msg(self.stopline)
        self.stoplines_pub.publish(all_stoplines)
        return 
    
    # ROS
    def make_waypoints_msg(self, waypoints, possible_change_direction):
        waypoints_msg = PoseArray()
        waypoints_msg.header.frame_id = possible_change_direction

        for i in range(len(waypoints)-1):
            pose = Pose()
            pose.position.x = waypoints[i][0]
            pose.position.y = waypoints[i][1]

            # 방향 계산
            yaw = np.arctan2(waypoints[i+1][1] - waypoints[i][1], waypoints[i+1][0] - waypoints[i][0])
            quaternion = quaternion_from_euler(0, 0, yaw)
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
        
            waypoints_msg.poses.append(pose)
        return waypoints_msg
    
    # ROS
    def make_way_msg(self, ways_for_visualize):
        ways = PoseArray()
        ways.header.frame_id = "utm"
        ways.header.stamp = rospy.Time.now()

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
                quaternion = quaternion_from_euler(0, 0, yaw)

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
    
    # ROS
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
    
    # ROS
    def make_stopline_msg(self, stopline_for_visualize):
        stoplines = MarkerArray()
        
        for stopline_id, stopline_node in stopline_for_visualize.items():
            
            positions = [self.stopline_nodes[v] for v in stopline_node]
            
            stopline = Marker()
            stopline.header.frame_id = "utm"
            stopline.type = Marker.LINE_STRIP
            stopline.action = Marker.ADD
            stopline.id = stopline_id
            
            stopline.scale.x = 0.1
            
            stopline.color.a = 1.0
            stopline.color.r = 1.0
            stopline.color.g = 0.0
            stopline.color.b = 0.0
            
            for position in [positions[0], positions[-1]]:
                p = Point()
                p.x = position[0]
                p.y = position[1]
                stopline.points.append(p)
                
            stoplines.markers.append(stopline) 
 
        return stoplines

    
if __name__ == "__main__":
    try:
        # ROS
        gpp_node = GlobalPathPlanning()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass