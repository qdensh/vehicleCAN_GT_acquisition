cmake_minimum_required(VERSION 2.8.3)
project(gps_rtk_msgs)

set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_COMPILER "g++")
set( CMAKE_CXX_FLAGS "-std=c++11 -o3" )

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation geometry_msgs  nav_msgs sensor_msgs)

generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs sensor_msgs)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
		${PROJECT_SOURCE_DIR}/src
  		${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/gps_rtk_msgs.cpp src/lib_src/gps_rkt_worker.cpp src/lib_src/an_packet_protocol.cpp src/lib_src/rs232.cpp src/lib_src/spatial_packets.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}   pthread)