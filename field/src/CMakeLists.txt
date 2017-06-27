cmake_minimum_required (VERSION 2.6)

#link_directories(/usr/include/apriltags/build/lib)
#link_libraries(apriltags)

# Set compiler to use c++ 11 features
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 or C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2") #Set the compiler to optimize to level 2

# Put all binary files into /bin and libraries into /lib
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(apriltags)
include_directories(${CMAKE_SOURCE_DIR}/apriltags)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(field Field.cpp Cam.cpp Robot.cpp Pose.cpp mqpif.cpp)
target_link_libraries(field apriltags ${OpenCV_LIBS}) #-pg) #-fopenmp)
