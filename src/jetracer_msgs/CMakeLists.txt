cmake_minimum_required(VERSION 3.8)
project(jetracer_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorState.msg"
  "msg/MotorCommand.msg"
  "msg/ServoState.msg"
  "msg/ServoCommand.msg"
  "msg/SystemStatus.msg"
  "msg/BatteryState.msg"
  "msg/WheelSpeeds.msg"
  "srv/Calibrate.srv"
  "srv/SetMotorPID.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
