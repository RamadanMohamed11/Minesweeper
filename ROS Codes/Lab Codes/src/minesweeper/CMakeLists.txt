cmake_minimum_required(VERSION 3.5)
project(minesweeper)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the custom message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MineMapCoordinates.msg"
  DEPENDENCIES std_msgs
)




ament_export_dependencies(rosidl_default_runtime)
ament_package()
