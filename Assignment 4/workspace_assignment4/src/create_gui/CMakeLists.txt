cmake_minimum_required(VERSION 2.4.6)
project(create_gui)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)
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
	localization
	rbo_create
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)

#add external wxwidgets library
find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories(${wxWidgets_INCLUDE_DIRS})


catkin_package(
  CATKIN_DEPENDS
  rbo_create
  localization
)


#otherwise CMAKE complains:
if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)


add_executable(button_move
	src/button_move_wx.cpp
	src/create_move.cpp
	src/create_status.cpp
)
target_link_libraries(button_move ${catkin_LIBRARIES} ${wxWidgets_LIBRARIES})

add_executable(map_view
	src/map_view.cpp
	src/ScrolledImageComponent.cpp
)
target_link_libraries(map_view ${catkin_LIBRARIES} ${wxWidgets_LIBRARIES})

add_executable(map_planner
	src/map_planner.cpp
	src/ScrolledImageComponent.cpp
)
target_link_libraries(map_planner ${catkin_LIBRARIES} ${wxWidgets_LIBRARIES})

set(CMAKE_CXX_FLAGS "-Wno-write-strings")  #we should fix that some time
