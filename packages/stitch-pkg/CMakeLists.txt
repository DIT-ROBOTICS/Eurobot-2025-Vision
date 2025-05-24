cmake_minimum_required(VERSION 3.8)
project(stitch_vulkan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Vulkan Directories
set(VULKAN_LIB_DIR $ENV{VULKAN_SDK}/lib)
set(VULKAN_INCLUDE_DIR $ENV{VULKAN_SDK}/include)
include_directories(${VULKAN_INCLUDE_DIR})
link_directories(${VULKAN_LIB_DIR})

# Vulkan Libraries
add_library(vulkan_processor
  src/vulkan_processor.cpp
)
target_link_libraries(vulkan_processor
  vulkan
  ${OpenCV_LIBS}
)
target_include_directories(vulkan_processor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${VULKAN_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
install(DIRECTORY include/
  DESTINATION include/
)
ament_target_dependencies(vulkan_processor
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)

# Vulkan node
add_executable(vulkan_node src/vulkan_node.cpp)
target_link_libraries(vulkan_node
  vulkan_processor
)
target_include_directories(vulkan_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(vulkan_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)

install(TARGETS
  vulkan_node
  vulkan_processor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
