#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pyproj
import numpy as np

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion, Point, PoseArray
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
import tf.transformations

def latlon_to_utm(lat, lon):
    proj = '+proj=utm +zone=52 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
    latlon_to_utm = pyproj.Proj(proj, preserve_units=True)
    return latlon_to_utm(lon, lat)

def euler_from_quaternion(q):
    """
    쿼터니언을 Euler angles로 변환 (roll, pitch, yaw 반환)
    """
    x, y, z, w = q.x, q.y, q.z, q.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)

    return roll_x, pitch_y, yaw_z

def transform_local_to_global(position, orientation, local_point):
    """
    차량의 global 위치(position)와 방향(orientation)을 이용해 local 좌표(local_point)를 global 좌표로 변환
    """
    # 쿼터니언을 Euler angles로 변환 (yaw만 사용)
    _, _, yaw = euler_from_quaternion(orientation)

    # 회전 변환 행렬 (2D 회전만 고려)
    rotation_matrix = np.array([
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw), np.cos(yaw)]
    ])

    # 로컬 좌표를 global로 변환
    local_xy = np.array([local_point[0], local_point[1]])
    global_xy = np.dot(rotation_matrix, local_xy) + np.array([position.x, position.y])

    # z 좌표는 변환 없이 그대로 사용
    global_z = local_point[2] + position.z

    return np.array([global_xy[0], global_xy[1], global_z])


class MapvizTF(object):
    def __init__(self):
        # self.location = None
        yaw = np.radians(-1.4440643432812905)
        # yaw = np.radians(-20)
        self.rot_offset = tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.rot_offset_quat = Quaternion(x=self.rot_offset[0],
                                    y=self.rot_offset[1], 
                                    z=self.rot_offset[2], 
                                    w=self.rot_offset[3])
        
        # ROS
        rospy.init_node('autocar_tf')

        # world -> mapviz transform 
        self.flag_world_to_mapviz = False
        self.tf_br_w2m = tf2_ros.StaticTransformBroadcaster() # world to mapviz

        # world -> base_link transform
        self.tf_br_w2bl = tf2_ros.TransformBroadcaster() # world to base_link

        # Obstalces velodyne -> utm transform
        self.tf_bf_vldyn2w= tf2_ros.Buffer()
        self.tf_listener_vldyn2w = tf2_ros.TransformListener(self.tf_bf_vldyn2w)

        self.local_origin_sub = rospy.Subscriber('/local_xy_origin', PoseStamped, self.callback_local_origin)
        self.gloabl_location_sub = rospy.Subscriber('/location', Odometry, self.callback_global_location)

        self.obstalces_sub = rospy.Subscriber('/adaptive_clustering/markers', MarkerArray, self.callback_obstacles)
        self.obstalces_utm_pub = rospy.Publisher('/obstacles_utm', PoseArray, queue_size=10)

        self.delivery_spot_sub = rospy.Subscriber('/deliverysign_spot', PoseArray, self.callback_spot)
        self.delivery_spot_utm_sub = rospy.Publisher('/delivery_utm', PoseArray, queue_size=10)
        return
    
    def callback_global_location(self, global_location_msg):
        """
        차량의 global 위치를 저장하는 콜백 함수
        """
        self.location = {
            "position": global_location_msg.pose.pose.position,
            "orientation": global_location_msg.pose.pose.orientation
        }
        
        # world(utm) -> base_link
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"

        t.transform.translation = global_location_msg.pose.pose.position
        t.transform.rotation = global_location_msg.pose.pose.orientation

        # Send the transformation
        self.tf_br_w2bl.sendTransform(t)
        return

    def callback_obstacles(self, clusters_msg):
        """
        장애물의 local 좌표를 global 좌표로 변환하는 콜백 함수
        """
        if not hasattr(self, 'location'):
            print("Global 위치 정보가 없습니다.")
            return
        
        utm_pose_array = PoseArray()
        utm_pose_array.header.frame_id = 'utm'
        utm_pose_array.header.stamp = rospy.Time.now()

        for cluster_msg in clusters_msg.markers:
            # 장애물의 local 좌표 계산
            points = np.array([(point.x, point.y, point.z) for point in cluster_msg.points])
            center = np.average(points, axis=0)  # x, y, z
            center[0] += 1.04 # base_link <-> velodyne

            current_orientation = self.location["orientation"]
            current_orientation_list = [
                                        current_orientation.x,
                                        current_orientation.y,
                                        current_orientation.z,
                                        current_orientation.w
                                        ]
            
            loc_orientation_offset_list = tf.transformations.quaternion_multiply(current_orientation_list, self.rot_offset)

            loc_orientation_offset = Quaternion(
                                    x=loc_orientation_offset_list[0], 
                                    y=loc_orientation_offset_list[1], 
                                    z=loc_orientation_offset_list[2], 
                                    w=loc_orientation_offset_list[3]
                                    )
            
            global_center = transform_local_to_global(
                self.location["position"],
                loc_orientation_offset,
                center
            )

            # 변환된 global 좌표를 Pose로 저장
            pose = PoseStamped()
            pose.header.frame_id = 'utm'
            pose.pose.position.x = global_center[0]
            pose.pose.position.y = global_center[1]
            pose.pose.position.z = global_center[2]

            utm_pose_array.poses.append(pose.pose)

        self.obstalces_utm_pub.publish(utm_pose_array)
        return

    def callback_local_origin(self, local_origin):
        if self.flag_world_to_mapviz == False:
            # world -> mapviz
            lat = local_origin.pose.position.y
            lot = local_origin.pose.position.x
            world_x, world_y = latlon_to_utm(lat, lot)

            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id = "mapviz"
        
            t.transform.translation = Point(x=world_x, y=world_y, z=0)

            t.transform.rotation = Quaternion(x=self.rot_offset[0], y=self.rot_offset[1], z=self.rot_offset[2], w=self.rot_offset[3])

            self.tf_br_w2m.sendTransform(t)

            print("static tf published")
            self.flag_world_to_mapviz = True

            self.local_origin_sub.unregister()
    
    def callback_spot(self, spot_pose_array_msg):
        if not hasattr(self, 'location'):
            print("Delivery spot 받음 / Localization 안받음.")
            return
        
        print("주차 좌표 변환")
        utm_pose_array = PoseArray()
        utm_pose_array.header.frame_id = 'utm'
        utm_pose_array.header.stamp = rospy.Time.now()

        for spot_pose in spot_pose_array_msg.poses:
            center = [spot_pose.position.x, spot_pose.position.y, 0.0]
            center[0] += 1.04 # base_link <-> velodyne

            current_orientation = self.location["orientation"]
            current_orientation_list = [
                                        current_orientation.x,
                                        current_orientation.y,
                                        current_orientation.z,
                                        current_orientation.w
                                        ]
            
            loc_orientation_offset_list = tf.transformations.quaternion_multiply(current_orientation_list, self.rot_offset)

            loc_orientation_offset = Quaternion(
                                    x=loc_orientation_offset_list[0], 
                                    y=loc_orientation_offset_list[1], 
                                    z=loc_orientation_offset_list[2], 
                                    w=loc_orientation_offset_list[3]
                                    )
            
            global_center = transform_local_to_global(
                self.location["position"],
                loc_orientation_offset,
                center
            )

            # ë³€í™˜ëœ global ì¢Œí‘œë¥¼ Poseë¡œ ì €ìž¥
            pose = PoseStamped()
            pose.header.frame_id = 'utm'
            pose.pose.position.x = global_center[0]
            pose.pose.position.y = global_center[1]

            utm_pose_array.poses.append(pose.pose)

        self.delivery_spot_utm_sub.publish(utm_pose_array)

        return
    
if __name__ == "__main__":
    try:
        # ROS
        mapviz_tf = MapvizTF()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass