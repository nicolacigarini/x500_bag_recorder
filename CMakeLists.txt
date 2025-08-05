cmake_minimum_required(VERSION 3.8)
project(x500_bag_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rosbag2_cpp)
find_package(sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(bag_recorder src/x500_bag_recorder_node.cpp)
ament_target_dependencies(bag_recorder rclcpp rosbag2_cpp sensor_msgs px4_msgs)

install(TARGETS 
  bag_recorder
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  config
  launch
  DESTINATION share/${PROJECT_NAME})


ament_package()
