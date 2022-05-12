cmake_minimum_required(VERSION 2.8.3)
project(rtsp_camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  std_msgs
  sensor_msgs
)

find_package(OpenCV)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rtsp_camera_lib
  CATKIN_DEPENDS roscpp cv_bridge image_transport sensor_msgs std_msgs
  DEPENDS OpenCV
)

catkin_add_env_hooks(25.rtsp-camera SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(rtsp_camera_lib
  src/rtsp_camera.cpp
)

target_link_libraries(rtsp_camera_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(rtsp_camera_lib ${catkin_EXPORTED_TARGETS})

add_executable(rtsp_camera_node src/main.cpp)
target_link_libraries(rtsp_camera_node rtsp_camera_lib)
add_dependencies(rtsp_camera_node rtsp_camera_lib)

#############
## Install ##
#############

install(TARGETS rtsp_camera_lib rtsp_camera_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
