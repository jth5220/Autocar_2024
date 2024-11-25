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

class MapvizTF(object):
    def __init__(self):
        yaw = np.radians(-1.4440643432812905)
        # yaw = np.radians(-20)
        self.rot_offset = tf.transformations.quaternion_from_euler(0, 0, yaw)
        
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

    def callback_obstacles(self, clusters_msg):
        target_frame = 'world'
        try:
            # mapviz 프레임으로의 변환 탐색
            self.transformer = self.tf_bf_vldyn2w.lookup_transform(target_frame, 'velodyne', rospy.Time(0),rospy.Duration(0.1))

            utm_pose_array = PoseArray()
            utm_pose_array.header.frame_id = 'utm'
            utm_pose_array.header.stamp = rospy.Time.now()
            
            for cluster_msg in clusters_msg.markers:
                points = np.array([(point.x, point.y, point.z) for point in cluster_msg.points])
                center = np.average(points, axis=0) # x, y, z

                pose = PoseStamped()
                pose.header = 'velodyne'
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = center
                pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, self.transformer)

                utm_pose_array.poses.append(pose_transformed.pose)
            
            print(utm_pose_array)
            print("###"*20)
            self.obstalces_utm_pub.publish(utm_pose_array)
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print("world <-> base_link 또는 base_link <-> velodyne 좌표 변환 발행 X")

        return
    
    def callback_spot(self, spot_pose_array_msg):
        target_frame = 'world'
        try:
            # mapviz 프레임으로의 변환 탐색
            self.transformer = self.tf_bf_vldyn2w.lookup_transform(target_frame, 'velodyne', rospy.Time(0),rospy.Duration(1.0))

            utm_pose_array = PoseArray()
            utm_pose_array.header.frame_id = 'utm'
            utm_pose_array.header.stamp = rospy.Time.now()
            
            for spot_pose in spot_pose_array_msg.poses:
                pose = PoseStamped()
                pose.header = 'velodyne'
                pose.pose = spot_pose
                pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, self.transformer)

                utm_pose_array.poses.append(pose_transformed.pose)
            
            self.delivery_spot_utm_sub.publish(utm_pose_array)
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print("world <-> base_link 또는 base_link <-> velodyne 좌표 변환 발행 X")

        return
    
if __name__ == "__main__":
    try:
        # ROS
        mapviz_tf = MapvizTF()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass