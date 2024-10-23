cmake_minimum_required(VERSION 3.5)
project(my_ros2_package)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Install Python files
install(PROGRAMS
  scripts/camera_image_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})


ament_package()
