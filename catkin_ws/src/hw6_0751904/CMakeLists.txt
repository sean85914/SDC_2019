cmake_minimum_required(VERSION 2.8.3)
project(hw6_0751904)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  sensor_msgs
  std_msgs
  tf
  pcl_ros
  pcl_conversions
)

#find_package(Boost REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(hw6_node src/hw6_node.cpp)
target_link_libraries(hw6_node ${catkin_LIBRARIES})

add_executable(icp_slam src/icp_slam.cpp)
target_link_libraries(icp_slam ${catkin_LIBRARIES})

add_executable(save_pcd src/save_pcd.cpp)
target_link_libraries(save_pcd ${catkin_LIBRARIES})
