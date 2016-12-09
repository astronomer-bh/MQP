cmake_minimum_required (VERSION 2.6)

link_directories(/usr/include/apriltags/build/lib)
link_libraries(apriltags)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(field Field.cpp Cam.cpp Robot.cpp Pose.cpp)

target_link_libraries(field ${OpenCV_LIBS}) #-pg) #-fopenmp)
