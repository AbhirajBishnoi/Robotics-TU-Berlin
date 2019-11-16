cmake_minimum_required(VERSION 2.8.7)
PROJECT(tutorialPlan)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Modules)
set(CMAKE_CXX_STANDARD 11)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Boost REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED) 
FIND_PACKAGE(SoQt REQUIRED) 
find_package(RL COMPONENTS KIN REQUIRED)
find_package(RL COMPONENTS PLAN REQUIRED)
find_package(RL COMPONENTS SG REQUIRED)
#find_package(PCL 1.7 REQUIRED)

if(CMAKE_SIZEOF_VOID_P EQUAL 4)
	add_definitions(-DEIGEN_DONT_ALIGN)
endif(CMAKE_SIZEOF_VOID_P EQUAL 4)

SET(
	HDRS
        qt_visualization/QtWindow.h
        qt_visualization/QtViewer.h
        qt_visualization/QtPlanningThread.h
	RrtConConBase.h
	TutorialPlanSystem.h
        YourPlanner.h
	YourSampler.h
)

SET(
	SRCS
        qt_visualization/QtWindow.cpp
        qt_visualization/QtViewer.cpp
        qt_visualization/QtPlanningThread.cpp
	RrtConConBase.cpp
	tutorialPlan.cpp
	TutorialPlanSystem.cpp
        YourPlanner.cpp
	YourSampler.cpp
)

QT4_WRAP_CPP(
        MOC_SRCS
        qt_visualization/QtWindow.h
        qt_visualization/QtViewer.h
        qt_visualization/QtPlanningThread.h
)

ADD_EXECUTABLE(
	tutorialPlan
	${HDRS}
	${SRCS}
    ${MOC_SRCS}
)

#target_compile_definitions(
add_definitions(
	"" ${QT_DEFINITIONS}
	-DSOQT_DLL
	-frounding-math
#		${PCL_DEFINITIONS}
)

#target_include_directories(
include_directories(
	${QT_INCLUDES}
	${EIGEN_INCLUDE_DIRS}
#	${PCL_INCLUDE_DIRS}
	${RL_DIR}/../../include
)

#link_directories(${PCL_LIBRARY_DIRS})

TARGET_LINK_LIBRARIES(
	tutorialPlan
	${RL_LIBRARIES}
#	${PCL_LIBRARIES}
	${QT_QTCORE_LIBRARY}
	${QT_QTGUI_LIBRARY}
	${QT_QTOPENGL_LIBRARY}
	${SOQT_LIBRARY_RELEASE}
	${COIN_LIBRARY_RELEASE}
)

