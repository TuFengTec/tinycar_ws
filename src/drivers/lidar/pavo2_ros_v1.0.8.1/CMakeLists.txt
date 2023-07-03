cmake_minimum_required(VERSION 2.8.3)
project(pavo2_ros)

set(pavo2_SDK_PATH "./sdk")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

file(GLOB pavo2_SDK_SRC
  "${pavo2_SDK_PATH}/src/*.cpp"
  "${pavo2_SDK_PATH}/src/impl/unix/*.cpp"
)


message("pavo2_SDK_SRC:${pavo2_SDK_SRC}")

set(ROS_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  sensor_msgs
  nav_msgs
  tf
  message_generation
  dynamic_reconfigure
  laser_geometry
  pcl_conversions
  pcl_ros
  rospy
)

find_package(PCL 1.7 REQUIRED)
 
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(
   cfg/pavo2_ros_cfg.cfg
)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pavo2_ros
#  CATKIN_DEPENDS rosconsole roscpp sensor_msgs
#  DEPENDS system_lib
)


###########
## Build ##
###########

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${pavo2_SDK_PATH}/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pavo2_scan_node src/pavo2_scan_node.cpp  ${pavo2_SDK_SRC})
target_link_libraries(pavo2_scan_node 
  ${catkin_LIBRARIES}
 )

# add_executable(my_pcl_node src/My_Filter.cpp src/my_pcl_node.cpp ${pavo2_SDK_SRC})
# target_link_libraries(my_pcl_node 
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#  )

add_executable(pavo2_pcd_node src/pavo2_pcd_node.cpp ${pavo2_SDK_SRC})

target_link_libraries(pavo2_pcd_node
  ${catkin_LIBRARIES}
)


# add_executable(odom src/odom.cpp ${pavo2_SDK_SRC})
# target_link_libraries(odom ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS pavo2_scan_node pavo2_pcd_node 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY launch rviz sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
