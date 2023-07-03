cmake_minimum_required(VERSION 3.0.2)
project(ins_msg)

SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
)

add_message_files(
  FILES
  ImuMsg.msg
  GnssMsg.msg
  GnssAjMsg.msg
  InsMsg.msg
  UwbMsg.msg
  UwbTdoaPosMsg.msg
  ByteVector.msg
  WheelSpeedMsg.msg
)

generate_messages(
 DEPENDENCIES
 std_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime
)

