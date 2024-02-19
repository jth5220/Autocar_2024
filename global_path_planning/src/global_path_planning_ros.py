#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from global_path_planning import GlobalPathPlanning
from keyboard_input import KeyboardInput

import numpy as np
import time
import pyproj

import rospy
import rospkg
from geometry_msgs.msg import PointStamped, PoseArray, Pose, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import tf.transformations as tf_trans

# 1. ways -> way -> node 를 marker로 시각화
# 2. /clicked_point 토픽 활용하여 원하는 Global path 추출

msg = """
======= Global path planning =========

Left mouse click: Make the global path

q: Delete the last added ways

w: change the selecting mode

Backspace: Delete all ways

======================================
"""

class GlobalPathPlanningRos():
    def __init__(self):
        rospack = rospkg.RosPack()
        osm_file_path = rospack.get_path('global_path_planning') + "/osm_files"
        osm_file_list = ["hitech2_LINK.osm", "hitech2_INTERSECTION_LINK.osm", "hitech_MISSION.osm"]

        self.global_path_planning = GlobalPathPlanning(osm_file_path, osm_file_list)

        # location (not corrected)
        self.location_not_corrected = None
        self.location_corrected = None

        # Keyboard Input
        self.keyboard_input = KeyboardInput(rate=0.05)
        self.event_list = {'\x7f': self.reset_selected_ways, # '\x7f': backspace
                           'q': self.remove_target_ways,     
                            '\x03': self.shutdown,          # '\x03': Ctrl + 'c'
                            'w': self.toggle_mode}           

        # Attributes
        self.is_published_once = False
        self.is_selecting_mode = True
        self.prev_way = None

        # ROS
        rospy.init_node('global_path_planning')
        
        self.clicked_point_sub = rospy.Subscriber('/clicked_point', PointStamped, self.callback_clicked_point)
        self.location_not_corrected_sub = rospy.Subscriber('/location_no_correction', Odometry, self.callback_not_corrected_location)
        self.location_corrected_sub = rospy.Subscriber('/location_corrected', Odometry, self.callback_corrected_location)

        self.way_pub = rospy.Publisher('/ways', PoseArray, queue_size=10) # 모든 ways
        self.mission_areas_pub = rospy.Publisher('/mission_areas', MarkerArray, queue_size=10) # 미션구역
        self.closest_way_pub = rospy.Publisher('/closest_way', PoseArray, queue_size=10) # 해당 지점에서 제일 가까운 way
        self.candidate_ways_pub = rospy.Publisher('/candidates_ways', PoseArray, queue_size=10) # 제일 가까운 way 다음의 후보 ways
        self.selected_ways_pub = rospy.Publisher('/selected_ways', PoseArray, queue_size=10) # target ways
        self.lateral_error_pub = rospy.Publisher('/ct_error_global', Float32, queue_size=10)

        self.timer_for_key_input = rospy.Timer(rospy.Duration(0.1), self.callback_timer_for_key_input)
        self.timer_for_location_check = rospy.Timer(rospy.Duration(0.1), self.callback_timer_for_location_check)

        rospy.loginfo(msg)
        return
    
    def callback_not_corrected_location(self, location_msg):
        self.location_not_corrected = (location_msg.pose.pose.position.x, location_msg.pose.pose.position.y)
        return
    
    def callback_corrected_location(self, location_msg):
        self.location_corrected = (location_msg.pose.pose.position.x, location_msg.pose.pose.position.y)
        return
    
    def callback_timer_for_key_input(self, event):
        # keyboard input
        key_input = self.keyboard_input.update()
        event = self.event_list.get(key_input, self.keyboard_input_is_none)
        event()
        return
    
    def callback_timer_for_location_check(self, event):
        if self.is_selecting_mode is True:
            # 모든 way와 미션구역에 대해 1번만 시각화
            if self.is_published_once == False and self.way_pub.get_num_connections() == 1:     
                rospy.loginfo("The all ways are published.")
                all_ways = self.global_path_planning.make_way_msg(self.global_path_planning.ways)
                self.way_pub.publish(all_ways)

                all_mission_areas = self.global_path_planning.make_mission_msg(self.global_path_planning.mission)
                self.mission_areas_pub.publish(all_mission_areas)

                self.is_published_once = True
            return
        
        if len(self.global_path_planning.selected_ways) == 0 or self.location_corrected is None:
            return
        
        cur_way = self.global_path_planning.update_current_way(self.location_corrected)

        closest_waypoints, _, closest_waypoint = self.global_path_planning.find_closest_nodes(cur_way, self.location_corrected)

        global_ct_error = self.global_path_planning.calculate_global_ct_error(closest_waypoint, self.location_not_corrected)    
        
        lateral_error_msg = Float32()
        lateral_error_msg.data = global_ct_error
        self.lateral_error_pub.publish(lateral_error_msg)
        
        # currnet way가 바뀌면 출력
        if cur_way != self.prev_way:
            self.prev_way = cur_way
            print(f"The currnet way: {cur_way}")

        return
    
    def callback_clicked_point(self, msg):
        if self.is_selecting_mode is False:
            return
        
        # clicked point msg (ll frame)
        lon, lat = msg.point.x, msg.point.y

        # convert the point msg into the utm frame
        clicked_point = self.latlon_to_utm(lat, lon)
        self.global_path_planning.update_clicked_point(clicked_point) # clicked_point 새로 업데이트
        
        # calculate and visualize the closest way
        closest_way_about_cur_pos = self.global_path_planning.find_closest_way(clicked_point)
        closest_way_msg = self.global_path_planning.make_way_msg([closest_way_about_cur_pos])
        self.closest_way_pub.publish(closest_way_msg)

        # calculate and visualize the candidate ways
        candidate_ways = self.global_path_planning.find_candidate_ways(closest_way_about_cur_pos)
        candidate_ways_msg = self.global_path_planning.make_way_msg(candidate_ways)
        self.candidate_ways_pub.publish(candidate_ways_msg)

        # calculate and visualize the optimal ways
        self.global_path_planning.update_selected_ways() # 업데이트된 clicked_point를 기반으로 selected_ways를 업데이트
        selected_ways_msg = self.global_path_planning.make_way_msg(self.global_path_planning.selected_ways)
        self.selected_ways_pub.publish(selected_ways_msg)
        
        rospy.loginfo('The way has been added.')
        return
    
    def reset_selected_ways(self):
        if self.is_selecting_mode is False:
            return
        
        self.global_path_planning.reset_selected_ways()

        # selected ways 업데이트
        selected_ways_msg = self.global_path_planning.make_way_msg(self.global_path_planning.selected_ways)
        self.selected_ways_pub.publish(selected_ways_msg)

        # closest_way와 candidtae_ways 업데이트
        closest_way_msg = self.global_path_planning.make_way_msg([])
        self.closest_way_pub.publish(closest_way_msg)

        candidate_ways_msg = self.global_path_planning.make_way_msg([])
        self.closest_way_pub.publish(candidate_ways_msg)

        rospy.loginfo('The all ways has been deleted.')
        return
    
    def remove_target_ways(self):
        if self.is_selecting_mode is False:
            return
        
        self.global_path_planning.remove_target_ways()

        # selected ways marker 업데이트
        selected_ways_msg = self.global_path_planning.make_way_msg(self.global_path_planning.selected_ways)
        self.selected_ways_pub.publish(selected_ways_msg)

        rospy.loginfo('The last added ways has been deleted.')
        return
    
    def toggle_mode(self):
        if self.is_selecting_mode is False:
            self.is_selecting_mode = True
            rospy.loginfo(f'You can select the way. (mode: {self.is_selecting_mode})')
        else:
            self.is_selecting_mode = False
            rospy.loginfo(f'Selection mode has been finished. (mode: {self.is_selecting_mode})')
        return
    
    @staticmethod
    def shutdown():
        rospy.signal_shutdown("Ctrl+C has been pressed.")

    @staticmethod
    def keyboard_input_is_none():
        return
    
    @staticmethod
    def latlon_to_utm(lat, lon):
        proj = '+proj=utm +zone=52 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
        latlon_to_utm = pyproj.Proj(proj, preserve_units=True)
        return latlon_to_utm(lon, lat)
    
if __name__ == "__main__":
    try:
        # ROS
        global_path_planning_ros = GlobalPathPlanningRos()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass