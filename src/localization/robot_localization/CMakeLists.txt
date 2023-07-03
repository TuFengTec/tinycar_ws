cmake_minimum_required(VERSION 3.0.2)
project(robot_localization)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  diagnostic_updater
  eigen_conversions
  geographic_msgs
  geometry_msgs
  message_filters
  message_generation
  nav_msgs
  nodelet
  roscpp
  roslint
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP yaml-cpp)


# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

# Attempt to find Eigen using its own CMake module.
# If that fails, fall back to cmake_modules package.
find_package(Eigen3)
set(EIGEN_PACKAGE EIGEN3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  set(EIGEN_PACKAGE Eigen)
endif()

set(CMAKE_CXX_STANDARD 14)

if(NOT MSVC)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Werror")
endif()
add_definitions(-DEIGEN_NO_DEBUG -DEIGEN_MPL2_ONLY)

set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references")
roslint_cpp()

###################################
## catkin specific configuration ##
###################################
add_service_files(
  FILES
    GetState.srv
    SetDatum.srv
    SetPose.srv
    SetUTMZone.srv
    ToggleFilterProcessing.srv
    FromLL.srv
    ToLL.srv
)

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
    ekf_localization_nodelet
    filter_base
    filter_utilities
    navsat_transform
    navsat_transform_nodelet
    ros_filter
    ros_filter_utilities
    robot_localization_estimator
    ros_robot_localization_listener
    ukf
    ukf_localization_nodelet
  CATKIN_DEPENDS
    cmake_modules
    diagnostic_msgs
    diagnostic_updater
    eigen_conversions
    geographic_msgs
    geometry_msgs
    message_filters
    message_runtime
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
  DEPENDS
    ${EIGEN_PACKAGE}
    GeographicLib
    YAML_CPP
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

# Library definitions
add_library(filter_utilities src/filter_utilities.cpp)
add_library(filter_base src/filter_base.cpp)
add_library(ekf src/ekf.cpp)
add_library(ukf src/ukf.cpp)
add_library(robot_localization_estimator src/robot_localization_estimator.cpp)
add_library(ros_robot_localization_listener src/ros_robot_localization_listener.cpp)
add_library(ros_filter_utilities src/ros_filter_utilities.cpp)
add_library(ros_filter src/ros_filter.cpp)
add_library(navsat_transform src/navsat_transform.cpp)
add_library(ekf_localization_nodelet src/ekf_localization_nodelet.cpp)
add_library(ukf_localization_nodelet src/ukf_localization_nodelet.cpp)
add_library(navsat_transform_nodelet src/navsat_transform_nodelet.cpp)

# Executables
add_executable(ekf_localization_node src/ekf_localization_node.cpp)
add_executable(ukf_localization_node src/ukf_localization_node.cpp)
add_executable(navsat_transform_node src/navsat_transform_node.cpp)
add_executable(robot_localization_listener_node src/robot_localization_listener_node.cpp)

# Dependencies
add_dependencies(filter_base ${PROJECT_NAME}_gencpp)
add_dependencies(navsat_transform ${PROJECT_NAME}_gencpp)
add_dependencies(robot_localization_listener_node ${PROJECT_NAME}_gencpp)

# Linking
target_link_libraries(ros_filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(filter_base filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(ekf filter_base ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(ukf filter_base ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(ros_filter ekf ukf ros_filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(robot_localization_estimator filter_utilities filter_base ekf ukf ${EIGEN3_LIBRARIES})
target_link_libraries(ros_robot_localization_listener robot_localization_estimator ros_filter_utilities
  ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${YAML_CPP_LIBRARIES})
target_link_libraries(robot_localization_listener_node ros_robot_localization_listener ${catkin_LIBRARIES})
target_link_libraries(ekf_localization_node ros_filter ${catkin_LIBRARIES})
target_link_libraries(ekf_localization_nodelet ros_filter ${catkin_LIBRARIES})
target_link_libraries(ukf_localization_node ros_filter ${catkin_LIBRARIES})
target_link_libraries(ukf_localization_nodelet ros_filter ${catkin_LIBRARIES})
target_link_libraries(navsat_transform filter_utilities ros_filter_utilities ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${GeographicLib_LIBRARIES})
target_link_libraries(navsat_transform_node navsat_transform ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})
target_link_libraries(navsat_transform_nodelet navsat_transform ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})

#############
## Install ##
#############

