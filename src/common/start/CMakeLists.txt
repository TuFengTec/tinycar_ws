cmake_minimum_required(VERSION 2.8.3)
project(tinycar_start)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  geometry_msgs
  nav_msgs
  std_msgs
  tf
  ins_msg
  serial
  laser_geometry
  
)
find_package( PCL REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
# ulimit -c unlimited
# ssh firefly@192.168.1.76
add_compile_options(-O3 -std=c++17)    #-g -> core dumped

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES huanyu_robot_start
#  CATKIN_DEPENDS geometry_msgs message_generation nav_msgs nav_msgs roscpp roslib rospy sensor_msgs std_msgs tf
CATKIN_DEPENDS roscpp roslib rospy  std_msgs nav_msgs geometry_msgs

)

# defind robot status massage
# command atkin_make -DCATKIN_WHITELIST_PACKAGES="new_msg"
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(
    include
    include/${PROJECT_NAME}
)

# add_dependencies(odom_node ins_driver_gencpp)
add_executable(link_node src/link_node.cpp)
target_link_libraries(link_node ${catkin_LIBRARIES})


add_executable(robot_pose_publisher src/robot_pose_publisher.cpp)
target_link_libraries(robot_pose_publisher ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

add_executable(cmdRateBoost src/cmdRateBoost.cpp)
target_link_libraries(cmdRateBoost ${catkin_LIBRARIES})