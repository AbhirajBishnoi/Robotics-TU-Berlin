cmake_minimum_required(VERSION 2.4.6)
project(mapping)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslib
  std_msgs
  sensor_msgs
  nav_msgs
  tf
  message_filters
  std_srvs
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS   
  sensor_msgs
  nav_msgs
)


add_executable(mapper src/MappingNode.cpp)
target_link_libraries(mapper ${catkin_LIBRARIES})
