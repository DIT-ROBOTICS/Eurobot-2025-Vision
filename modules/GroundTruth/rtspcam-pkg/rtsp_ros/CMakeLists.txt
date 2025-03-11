cmake_minimum_required(VERSION 3.8)
project(rtsp_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED) 
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(camera_info_manager REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
)
link_directories(${GSTREAMER_LIBRARY_DIRS})
add_definitions(${GSTREAMER_CFLAGS_OTHER})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(rtsp_cam_node SHARED
  src/rtsp_cam_node.cpp
)

target_include_directories(rtsp_cam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(rtsp_cam_node
  ${OpenCV_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  gstapp-1.0 gstreamer-1.0 gobject-2.0 glib-2.0 
  yaml-cpp
)

ament_export_libraries(rtsp_cam_node)

ament_target_dependencies(rtsp_cam_node
  rclcpp
  rclcpp_components
  sensor_msgs
  OpenCV
  cv_bridge
  yaml-cpp
  camera_info_manager
)

rclcpp_components_register_nodes(rtsp_cam_node "rtsp_ros::CamNodePublisher")

install(
  TARGETS rtsp_cam_node
  EXPORT export_rtsp_cam_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  OpenCV
  cv_bridge
  yaml-cpp
  camera_info_manager
)

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
