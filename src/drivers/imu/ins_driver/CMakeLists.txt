cmake_minimum_required(VERSION 3.0.2)
project(ins_driver)

SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  ins_msg
  message_generation
)



## Build ##
find_package(Eigen3 3.1.0 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_package(
  INCLUDE_DIRS 

  CATKIN_DEPENDS roscpp roslib rospy  std_msgs geometry_msgs sensor_msgs nav_msgs
)

## LIB serial_driver
ADD_LIBRARY(serial_driver_libs SHARED ./src/serial.cpp)



## dgnss
ADD_LIBRARY(ins SHARED ./src/INS/ins.cpp ./src/INS/ring_buffer.c)
TARGET_LINK_LIBRARIES(ins serial_driver_libs )

ADD_EXECUTABLE(INS_node ./src/INS/ins_node.cpp)
TARGET_LINK_LIBRARIES(INS_node ${catkin_LIBRARIES} ins)

