# 단축키
alias camlane='cd ~/catkin_ws/src/sensor && python3 camera_ros_lane.py'

alias camdelivery='cd ~/catkin_ws/src/sensor && python3 camera_ros_delivery.py'

alias camtraffic='cd ~/catkin_ws/src/sensor && python3 camera_ros_traffic.py'

alias gps='roslaunch ublox_gps ublox_device.launch'

alias ntrip='roslaunch ntrip_ros ntrip_ros.launch'

alias imu='roslaunch xsens_mti_driver xsens_mti_node.launch'

alias lidar='roslaunch velodyne_pointcloud VLP16_points.launch'

alias mapviz='roslaunch mapviz mapviz.launch'

alias gpp='cd ~/catkin_ws/src/planning/global_path_planning/src && python3 global_path_planning.py'

alias loc='cd ~/catkin_ws/src/localization/src && python3 localization.py'

alias lane='cd ~/catkin_ws/src/perception/Ultrafast-Lane-Detection-Inference-Pytorch && python3 lanenet.py'

alias stop='cd ~/catkin_ws/src/localization/src && python3 stopline_detect.py'

alias cluster='roslaunch adaptive_clustering adaptive_clustering.launch'

alias frenet='cd ~/catkin_ws/src/planning/optimal_frenet_planning/src && python3 local_path_planning.py'

alias ptr='cd ~/catkin_ws/src/path_tracking/src && python3 path_tracking.py'

alias erp='rosrun erp_control erp42_ros.py'

alias enc='cd ~/catkin_ws/src/sensor && python3 encoder_to_vel.py'

alias dead='cd ~/catkin_ws/src/dead_reckoning && python3 dead_reckoning_utm.py'

alias tf='cd ~/catkin_ws/src/util/src && python3 autocar_tf.py'

alias remap='cd ~/catkin_ws/src/util/src && python3 topic_remap.py'

alias delivery='cd ~/catkin_ws/src/perception/yolo_detection/src && python3 delivery_detect.py'

alias ssf='cd ~/catkin_ws/src/perception/sensor_fusion/src && python3 sensor_fusion.py'

alias traffic='cd ~/catkin_ws/src/perception/yolo_detection/src && python3 trafficlight.py'

alias ptr='cd ~/catkin_ws/src/path_tracking/src && python3 path_tracking.py'
alias erp='rosrun erp_control erp42_ros.py'
