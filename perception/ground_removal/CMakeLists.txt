cmake_minimum_required(VERSION 3.0.2)
project(ground_removal)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
)

find_package(PCL REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(ground_removal src/ground_removal.cpp)
target_link_libraries(ground_removal ${catkin_LIBRARIES})


