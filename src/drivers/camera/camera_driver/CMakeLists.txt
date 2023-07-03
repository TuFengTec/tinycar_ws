cmake_minimum_required(VERSION 3.5)
project(camera_driver)




SET(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  message_runtime
  roscpp
  sensor_msgs
  std_msgs
)

#include_directories(${PROJECT_SOURCE_DIR}
#    ${GOOGLE_PROTOBUF_DIR}/include
#)

#link_directories(${GOOGLE_PROTOBUF_DIR}/lib/)

# camera lib
#INCLUDEPATH += /opt/GalaxyU3/Galaxy_camera/inc/
#LIBS += /opt/GalaxyU3/Galaxy_camera/lib/x86_64/libgxiapi.so

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
    CATKIN_DEPENDS 
    cv_bridge image_transport 
    message_generation 
    roscpp 
    sensor_msgs 
    std_msgs
)

# SET( DHCamera_INC_DIR "/opt/GalaxyU3/Galaxy_camera/inc/")
# SET( DHCamera_LIB_DIR "/opt/GalaxyU3/Galaxy_camera/lib/x86_64/")
# set( DHCamera_LIBS    "/opt/GalaxyU3/Galaxy_camera/lib/x86_64/libgxiapi.so")

SET( DHCamera_INC_DIR "./inc/")
SET( DHCamera_LIB_DIR "/opt/GalaxyU3/Galaxy_camera/lib/x86_64/")
# set( DHCamera_LIBS    "/home/firefly/workspace/tinycar/src/uwbee-minicar/minicar_drivers/camera_driver/lib/armv8/libgxiapi.so")
set( DHCamera_LIBS    "${CMAKE_CURRENT_SOURCE_DIR}/lib/armv8/libgxiapi.so")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${DHCamera_INC_DIR}
)

link_directories(
  ${DHCamera_LIB_DIR}/lib/
)

#file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
#file(GLOB_RECURSE DH_SRCS "src/camera_driver/DHCamera/*.cpp" )
#MESSAGE("include folder: "${DH_SRCS})

#file(GLOB_RECURSE ALL_SRCS   "src/*.cpp")

#MESSAGE("include folder: "${ALL_SRCS})

add_executable(camera_node 
  src/camera_node.cpp
  src/ICamController.cpp
  src/DHCamera2.cpp
  src/DHCameraROS.cpp
)
#add_executable(camera_node ${ALL_SRCS} )
add_dependencies(camera_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(camera_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${DHCamera_LIBS})

add_executable(cam_test_subscriber src/cam_test_subscriber.cpp)
add_dependencies(cam_test_subscriber ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(cam_test_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBS})