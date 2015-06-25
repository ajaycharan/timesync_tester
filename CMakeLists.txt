cmake_minimum_required(VERSION 2.8.3)
project(timesync_tester)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  message(STATUS "set compile flags for Linux system")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  message(STATUS "set compile flags for Darwin system")
  set(CMAKE_CXX_FLAGS "-std=c++11 -stdlib=libc++")
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  message(STATUS "set compile flags for Windows system")
  message(AUTHOR_WARNING "TODO ... compile flags for Windows")
else()
  message(SEND_ERROR "Unknown/unsupported(?) system")
endif()
   
find_package(catkin_simple REQUIRED)
catkin_simple()

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	cs_add_executable(timesync_server src/timesync_server.cpp)
endif()

cs_add_executable(timesync_client src/timesync_client.cpp)


cs_export()




