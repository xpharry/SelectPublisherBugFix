cmake_minimum_required(VERSION 2.8.3)
project(display_mobot_view)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  pcl_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package()

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(display_point_info src/display_point_info.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(display_point_info ${catkin_LIBRARIES})


