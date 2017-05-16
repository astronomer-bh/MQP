cmake_minimum_required (VERSION 2.6)

link_libraries(apriltags)

include_directories(/usr/include/eigen3)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

set(SOURCES Field.cpp Cam.cpp Robot.cpp Pose.cpp)
add_executable(field ${SOURCES})

target_link_libraries(field ${OpenCV_LIBS}) #-pg) #-fopenmp)
