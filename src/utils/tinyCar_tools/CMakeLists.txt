cmake_minimum_required(VERSION 3.0.2)
project(minicar_tools)

find_package(catkin REQUIRED COMPONENTS geometry_msgs rosconsole roscpp roscpp_serialization roslib rostime std_msgs std_srvs)

find_package(Qt5Widgets REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


catkin_package(CATKIN_DEPENDS geometry_msgs  std_msgs std_srvs)




add_executable(minicar_key tutorials/teleop_turtle_key2.cpp)
target_link_libraries(minicar_key ${catkin_LIBRARIES})
#add_dependencies(minicar_key turtlesim_gencpp)


