#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32
# from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Image

class TopicRemap():
    def __init__(self):
        self._set_ros()
        return
    
    def _set_ros(self):
        # ROS
        rospy.init_node('topic_remap')
        self.carla_gps_sub = rospy.Subscriber('/carla/ego_vehicle/gnss', NavSatFix, self.callback_carla_gps)
        self.carla_imu_sub = rospy.Subscriber('/carla/ego_vehicle/imu', Imu, self.callback_carla_imu)
        self.carla_speed_sub = rospy.Subscriber('/carla/ego_vehicle/speedometer', Float32, self.callback_carla_speed)
        self.carla_lidar_sub = rospy.Subscriber('carla/ego_vehicle/lidar', PointCloud2, self.callback_carla_lidar)
        self.carla_trafficlight_cam_sub = rospy.Subscriber('/carla/ego_vehicle/rgb_front/image', Image, self.callback_tl_cam)
        self.carla_delivery_cam_sub = rospy.Subscriber("/carla/ego_vehicle/rgb_right/image", Image, self.callback_dl_cam)

        self.erp_gps_pub = rospy.Publisher('/ublox_gps/fix', NavSatFix, queue_size=10)
        self.erp_imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.erp_speed_pub = rospy.Publisher('/encoder/speed', Float32, queue_size=10)
        self.erp_lidar_pub = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=10)
        self.erp_tl_cam_pub = rospy.Publisher('/image/traffic_light', Image, queue_size=10)
        self.erp_dl_cam_pub = rospy.Publisher('/image/delivery', Image, queue_size=10)
        return
    
    def callback_carla_gps(self, gps_msg):
        gps_msg.header.frame_id = 'gps'
        self.erp_gps_pub.publish(gps_msg)

    def callback_carla_imu(self, imu_msg):
        imu_msg.header.frame_id = 'base_link'
        # imu_msg.linear_acceleration.x = 0
        # imu_msg.linear_acceleration.y = 0
        # imu_msg.linear_acceleration.z = 0
        self.erp_imu_pub.publish(imu_msg)

    def callback_carla_speed(self, speed_msg):
        self.erp_speed_pub.publish(speed_msg)

    def callback_carla_lidar(self, lidar_msg):
        lidar_msg.header.frame_id = 'velodyne'
        self.erp_lidar_pub.publish(lidar_msg)

    def callback_tl_cam(self, img_msg):
        self.erp_tl_cam_pub.publish(img_msg)
    
    def callback_dl_cam(self, img_msg):
        self.erp_dl_cam_pub.publish(img_msg)
        
if __name__ == "__main__":
    try:
        # ROS
        tr = TopicRemap()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass