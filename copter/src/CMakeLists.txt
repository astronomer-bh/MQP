cmake_minimum_required (VERSION 2.8)

SET(PROJECT_NAME "copter")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -std=c++11 -pthread")

PROJECT(${PROJECT_NAME})

find_package(X11 REQUIRED)
find_package(Threads REQUIRED)

add_definitions(${CMAKE_CXX_FLAGS})

link_directories(/usr/include/apriltags/build/lib)
link_libraries(apriltags)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} Copter.cpp Field.cpp Cam.cpp Robot.cpp Pose.cpp)

target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS}) #-pg) #-fopenmp)
