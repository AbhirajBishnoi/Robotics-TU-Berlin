cmake_minimum_required (VERSION 2.8.11)
project (vision)

include_directories(BEFORE ${PROJECT_SOURCE_DIR}/../include /opt/pumasim/include)


if(CMAKE_BUILD_TYPE STREQUAL "RealCamera")
    add_definitions(-DSIM)
else()
    add_definitions(-DSIM -DUSE_SIMULATED_CAMERA)
endif()



## For compilation of  the control loop library:
add_library(controlDLL SHARED 
    controlDLL/control.cpp
)
target_link_libraries (controlDLL)



## For compilation of  the vision loop library:
find_package(OpenCV REQUIRED)
add_library(cv_main SHARED 
    cv/cv_main.cpp 
    cv/cv_control.cpp
)
target_link_libraries (cv_main ${OpenCV_LIBS})


##
## Use this to copy resources from SRC into the build directory
##
#file(COPY 
#    texture.raw
#    DESTINATION .
#)
