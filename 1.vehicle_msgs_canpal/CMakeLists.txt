cmake_minimum_required(VERSION 2.8.3)
project(vehicle_msgs_canpal)

set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_FLAGS_DEBUG "-g3")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation std_msgs nav_msgs sensor_msgs geometry_msgs rosbag)

 add_message_files(
   FILES
   VehicleStamped.msg
 )

generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs sensor_msgs)

catkin_package(
    CATKIN_DEPENDS 
    message_runtime
    std_msgs
    geometry_msgs 
    nav_msgs 
    sensor_msgs
)

include_directories(
	${PROJECT_SOURCE_DIR}/src
  	${catkin_INCLUDE_DIRS}
)
link_directories(${PROJECT_SOURCE_DIR}/src/lib/wiican/release)

add_executable(${PROJECT_NAME}_a27 src/vehicle_msgs_canpal_a27.cpp src/lib/dbc/a27/a27.cpp)
target_link_libraries(${PROJECT_NAME}_a27 ${catkin_LIBRARIES} WIICAN pthread)
add_dependencies(${PROJECT_NAME}_a27 ${PROJECT_NAME}_gencpp)

add_executable(${PROJECT_NAME}_c11 src/vehicle_msgs_canpal_c11.cpp src/lib/dbc/c11/c11.cpp)
target_link_libraries(${PROJECT_NAME}_c11 ${catkin_LIBRARIES} WIICAN pthread)
add_dependencies(${PROJECT_NAME}_c11 ${PROJECT_NAME}_gencpp)
