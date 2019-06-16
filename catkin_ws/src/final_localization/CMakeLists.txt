cmake_minimum_required(VERSION 2.8.3)
project(final_localization)
add_compile_options(-std=c++14)

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
find_package(GeographicLib REQUIRED)
set(CMAKE_BUILD_TYPE Release)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

add_executable(localization src/localization.cpp)
target_link_libraries(localization ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})
