cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# add_definitions(-std=c++11)
add_definitions(-std=c++14)  # this is required for PCL 1.10

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

# find_package(PCL 1.2 REQUIRED)
find_package(PCL 1.10 REQUIRED)  # my machine has PCL 1.10.0

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (quizCluster cluster.cpp ../../render/render.cpp)
target_link_libraries (quizCluster ${PCL_LIBRARIES})
