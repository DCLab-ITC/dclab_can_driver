cmake_minimum_required(VERSION 3.8)
project(can_driver_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_messages REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(can_driver_node src/can_driver_node.cpp src/canable_reclaim.cpp)
target_include_directories(can_driver_node PRIVATE include)
ament_target_dependencies(can_driver_node rclcpp custom_messages geometry_msgs ament_index_cpp)
install(TARGETS can_driver_node DESTINATION lib/${PROJECT_NAME})

# DCLab Link stays Python: the same file as the Python package's
install(PROGRAMS ../can_driver/can_driver/dclab_link.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_frames test/test_frames.cpp)
  target_include_directories(test_frames PRIVATE include)
  add_test(NAME test_frames COMMAND test_frames)
endif()

ament_package()
