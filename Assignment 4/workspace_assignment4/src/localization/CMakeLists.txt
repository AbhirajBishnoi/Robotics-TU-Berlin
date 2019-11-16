cmake_minimum_required(VERSION 2.4.6)
project(localization)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslib
  std_msgs
  sensor_msgs
  nav_msgs
  tf
  message_filters
  std_srvs
  message_generation
)


#adds the message definitions from msg/:
add_message_files(FILES
  PoseWithWeight.msg
  PoseWithWeightArray.msg
  PoseWithWeightStamped.msg
)

#adds the service definition files from srv/
add_service_files(FILES
  SetInitialPose.srv
)


#To compile the message and service definitions in msg/ and srv/:
generate_messages(DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  sensor_msgs
  nav_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)


add_executable(particlefilter
  src/ParticleFilterNode.cpp
  src/ParticleFilter.cpp
  src/Util.cpp
)
target_link_libraries(particlefilter ${catkin_LIBRARIES})

#makes sure the executable depends on the messages and services generated within this package
add_dependencies(particlefilter ${${PROJECT_NAME}_EXPORTED_TARGETS})
