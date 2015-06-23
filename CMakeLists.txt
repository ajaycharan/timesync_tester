cmake_minimum_required(VERSION 2.8.3)
project(timesync_tester)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11")

find_package(catkin_simple REQUIRED)
catkin_simple()

cs_add_executable(timesync_server src/timesync_server.cpp)
cs_add_executable(timesync_client src/timesync_client.cpp)


cs_export()




