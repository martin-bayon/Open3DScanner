cmake_minimum_required(VERSION 3.8)
project(open_3d_scanner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(colorize_pointcloud_node
  src/colorize_pointcloud_node.cpp
)

ament_target_dependencies(colorize_pointcloud_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  message_filters
  tf2
  tf2_ros
  tf2_eigen
)

target_link_libraries(colorize_pointcloud_node
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
)

target_include_directories(colorize_pointcloud_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

install(TARGETS
  colorize_pointcloud_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
