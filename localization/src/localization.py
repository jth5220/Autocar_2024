#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pyproj
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf

import rospy
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class Localization():
    def __init__(self):
        self.location = Odometry()
        self.location.header.frame_id = 'utm'

        self.location_corrected = Odometry()
        self.location_corrected.header.frame_id = 'utm'

        self.yaw_offset = 0 # radians
        self.global_yaw = 0
        self.lateral_offset = (0,0) # meters
        self.global_cte = 0
        self.lateral_error = 0

        self.br = tf.TransformBroadcaster()
        # ROS
        rospy.init_node('localization')
        
        self.gps_sub = rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.callback_gps)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.callback_imu)
        self.init_orientation_sub = rospy.Subscriber('/initial_global_pose', PoseWithCovarianceStamped, self.callback_init_orientation)

        self.local_cte_sub = rospy.Subscriber('/ct_error_local', Float32, self.callback_local_cte)
        self.global_sub = rospy.Subscriber('/ct_error_global', Float32, self.callback_global_cte)
        
        self.location_no_correction_pub = rospy.Publisher('/location_no_correction', Odometry, queue_size=10)
        self.location_corrected_pub = rospy.Publisher('/location_corrected', Odometry, queue_size=10)
        self.lateral_error_pub = rospy.Publisher('/lateral_error', Float32, queue_size=10)
        self.timer_location_publish = rospy.Timer(rospy.Duration(0.1), self.callback_timer_location_pub)
        return
    
    def callback_timer_location_pub(self, event):
        self.location_no_correction_pub.publish(self.location)
        self.location_corrected_pub.publish(self.location_corrected)
        # self.publish_tf_utm_to_bl()

        lateral_error_msg = Float32()
        lateral_error_msg.data = self.lateral_error
        self.lateral_error_pub.publish(lateral_error_msg)

        return
    
    # def publish_tf_utm_to_bl(self):
    #     t = TransformStamped()
    #     t.header.stamp = rospy.Time.now()
    #     t.header.frame_id = "utm_"
    #     t.child_frame_id = "base_link"
    #     t.transform.translation.x = self.location.pose.pose.position.x
    #     t.transform.translation.y = self.location.pose.pose.position.y
    #     t.transform.translation.z = self.location.pose.pose.position.z
    #     t.transform.rotation = self.location.pose.pose.orientation

    #     # Send the transformation
    #     self.br.sendTransformMessage(t)
    #     return
    
    def callback_gps(self, gps_msg):
        self.location.pose.pose.position.x, self.location.pose.pose.position.y = self.latlon_to_utm(gps_msg.latitude, gps_msg.longitude)

        self.location_corrected.pose.pose.position.x = self.location.pose.pose.position.x + self.lateral_offset[0]
        self.location_corrected.pose.pose.position.y = self.location.pose.pose.position.y + self.lateral_offset[1]
        return
    
    def callback_imu(self, imu_msg):
        local_yaw = euler_from_quaternion([imu_msg.orientation.x, imu_msg.orientation.y,\
                                          imu_msg.orientation.z, imu_msg.orientation.w])[2]
        global_yaw = local_yaw + self.yaw_offset
        self.location.pose.pose.orientation.x, self.location.pose.pose.orientation.y, \
        self.location.pose.pose.orientation.z, self.location.pose.pose.orientation.w = quaternion_from_euler(0, 0, global_yaw)

        self.location_corrected.pose.pose.orientation = self.location.pose.pose.orientation
        return
    
    def callback_init_orientation(self, init_pose_msg):
        global_yaw = euler_from_quaternion([init_pose_msg.pose.pose.orientation.x, init_pose_msg.pose.pose.orientation.y, \
                                           init_pose_msg.pose.pose.orientation.z, init_pose_msg.pose.pose.orientation.w])[2]
        
        local_yaw = euler_from_quaternion([self.location.pose.pose.orientation.x, self.location.pose.pose.orientation.y,\
                                          self.location.pose.pose.orientation.z, self.location.pose.pose.orientation.w])[2]

        self.yaw_offset += global_yaw - local_yaw
        return
    
    def callback_global_cte(self, global_cte_msg):
        self.global_cte = global_cte_msg.data
        print('global_cte: ', self.global_cte)
        return
    
    def callback_local_cte(self, local_cte_msg):
        local_cte = local_cte_msg.data
        lateral_error = local_cte - self.global_cte
        self.lateral_error = lateral_error
        global_yaw = euler_from_quaternion([self.location.pose.pose.orientation.x, self.location.pose.pose.orientation.y,\
                                          self.location.pose.pose.orientation.z, self.location.pose.pose.orientation.w])[2]
        
        # Calculate the direction perpendicular to global_yaw
        perpendicular_direction = global_yaw - np.pi/2  # This is in radians

        # Calculate the displacement in the x and y directions
        self.lateral_offset = (lateral_error * np.cos(perpendicular_direction), lateral_error * np.sin(perpendicular_direction))
        return
    
    @staticmethod
    def latlon_to_utm(lat, lon):
        proj = '+proj=utm +zone=52 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
        latlon_to_utm = pyproj.Proj(proj, preserve_units=True)
        return latlon_to_utm(lon, lat)
    
if __name__ == "__main__":
    try:
        # ROS
        localization = Localization()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass