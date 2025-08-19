cmake_minimum_required(VERSION 3.8)
project(edu_drive_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

add_executable(${PROJECT_NAME}_node
  src/edu_drive_node.cpp
  src/EduDrive.cpp
  src/MotorController.cpp
  src/RPiAdapterBoard.cpp
  src/RPiExtensionBoard.cpp
  src/PowerManagementBoard.cpp
  src/Odometry.cpp
  src/can/canprotocol.cpp
  src/can/SocketCAN.cpp
  src/can/SocketCANObserver.cpp
  src/Matrix.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_srvs
  sensor_msgs
  tf2_ros
  tf2
)

install(
  DIRECTORY
    launch
    parameter
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
