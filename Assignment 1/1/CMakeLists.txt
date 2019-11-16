cmake_minimum_required (VERSION 2.8.11)
project (vision)

set(CMAKE_BUILD_TYPE Debug)

include_directories(BEFORE ${PROJECT_SOURCE_DIR}/../include)

#creates a convenient link to pumasim in the build directory
add_custom_command(
    TARGET controlDLL POST_BUILD
    COMMAND ln -sf ../pumasim pumasim
)






## For compilation of  the control loop library:
add_library(controlDLL SHARED 
    controlDLL/control.cpp
)
target_link_libraries (controlDLL)





### For compilation of  the vision loop library:
#find_package(OpenCV REQUIRED)
#add_library(cv_main SHARED 
#    cv/cv_main.cpp 
#    cv/cv_control.cpp
#)
#target_link_libraries (cv_main ${OpenCV_LIBS})

