cmake_minimum_required(VERSION 3.8)
project(aruco_pipeline)
# === Set C++ standard ===
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_C_COMPILER_LAUNCHER ccache)
# set(CMAKE_CXX_COMPILER_LAUNCHER ccache)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Find dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
set(DEPS
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2 tf2_ros
  cv_bridge
  message_filters
  image_transport
  rclcpp_lifecycle
)

# === Library ===
add_library(aruco_detector STATIC
  src/aruco_detector.cpp
)
target_include_directories(aruco_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

add_library(aruco_transformer STATIC
  src/aruco_transformer.cpp
)
target_include_directories(aruco_transformer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(aruco_transformer
  tf2_geometry_msgs
  tf2 tf2_ros
)

# === Executable ===
add_executable(aruco_pipeline src/aruco_pipeline.cpp)
target_link_libraries(aruco_pipeline
  aruco_detector
  aruco_transformer
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(aruco_pipeline ${DEPS})

# === Install ===
install(TARGETS
  aruco_detector
  aruco_transformer
  aruco_pipeline
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# === Optional testing ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
