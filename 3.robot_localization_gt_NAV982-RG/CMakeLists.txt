cmake_minimum_required(VERSION 2.8.3)
project(robot_localization)

set(DEFAULT_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -g")
add_definitions(-std=c++11)
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE
            STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
            "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif ()
# OpenMP
FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  MESSAGE(STATUS "Found OpenMP")
ENDIF()

find_package(Sophus REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  diagnostic_updater
  geographic_msgs
  geometry_msgs
  message_filters
  message_generation
  nav_msgs
  roscpp
  roslint
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  ackermann_msgs
  
  roscpp
  std_msgs
  tf
  nav_msgs
  sensor_msgs
  geometry_msgs
  eigen_conversions
  tf_conversions
  random_numbers
  message_generation
  nodelet
  image_transport
  cv_bridge
  message_filters
  pcl_conversions
  pcl_ros
  std_srvs
  )

# Attempt to find Eigen using its own CMake module.
# If that fails, fall back to cmake_modules package.
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()


add_definitions(-DEIGEN_NO_DEBUG -DEIGEN_MPL2_ONLY)

roslint_cpp()

generate_messages(
  DEPENDENCIES
  geographic_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES 
    ekf
    filter_base
    filter_utilities
    navsat_transform
    ros_filter
    ros_filter_utilities
    ukf
  CATKIN_DEPENDS
    cmake_modules
    diagnostic_msgs
    diagnostic_updater
    geographic_msgs
    geometry_msgs
    message_filters
    message_runtime
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
  DEPENDS Eigen
)

###########
## Build ##
###########

include_directories(
  include ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

# Executables
add_executable(gt_tf_broadcaster src/gt_tf_broadcaster.cpp src/utils.cpp) # add-in

# Linking
target_link_libraries(gt_tf_broadcaster ${catkin_LIBRARIES} ${Sophus_LIBRARIES}) # add-in
