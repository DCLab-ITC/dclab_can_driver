cmake_minimum_required(VERSION 3.5)
project(custom_messages)

# Default to C99 for C and C++14 for C++
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define message files
set(msg_files
  "msg/DCLabMotorCommand.msg"
  "msg/DigitalAndAnalogFeedback.msg"
  "msg/DigitalAndSolenoidCommand.msg"
  "msg/EncoderFeedback.msg"
  "msg/PwmCommand.msg"
  "msg/RobomasterCommand.msg"
  "msg/RobomasterCurrentCommand.msg"
  "msg/RobomasterFeedback.msg"
  "msg/ServoCommand.msg"
  "msg/VescCommand.msg"
  "msg/VescStatusOne.msg"
  "msg/VescStatusTwo.msg"
  "msg/VescStatusThree.msg"
  "msg/VescStatusFour.msg"
  "msg/SwerveCommand.msg"
  "msg/SwerveFeedback.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package
ament_package()

