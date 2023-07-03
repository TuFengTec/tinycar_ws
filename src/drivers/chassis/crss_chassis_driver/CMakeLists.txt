cmake_minimum_required(VERSION 3.0.8)
project(crss_chassis_driver)

add_compile_options(-std=c++14)
add_compile_options(-fPIC)

SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++14 -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosmsg
  std_msgs
  message_generation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

################################################
## Declare ROS messages, services and actions ##
################################################

# add_service_files(
#   FILES
#     CanCtrlInfo.srv
# )

add_message_files(
  DIRECTORY msg
  FILES
    ChassisInfo.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
# catkin_package(
#   INCLUDE_DIRS include
#   CATKIN_DEPENDS 
#     roscpp
#     # vd_msgs
# )

###########
## Build ##
###########

link_directories(
  ${catkin_LIB_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
add_executable(can_node_nx src/can_node_nx.cpp)
target_link_libraries(can_node_nx pthread ${catkin_LIBRARIES})
# add_dependencies(can_node_nx ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} vd_msgs_generate_messages_cpp)
add_dependencies(can_node_nx ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
#endif()

#############
## Install ##
#############

#############
## Testing ##
#############

