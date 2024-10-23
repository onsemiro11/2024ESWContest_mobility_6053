cmake_minimum_required(VERSION 3.5)
project(travi_ui)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# 두 개의 실행 파일 설정
add_executable(travi_ui_node src/TRAVI_ui.cpp)
add_executable(keyboard_publisher src/keyboard_publisher.cpp)

# ament_target_dependencies로 ROS2 및 OpenCV 연결
ament_target_dependencies(travi_ui_node rclcpp std_msgs sensor_msgs OpenCV)
ament_target_dependencies(keyboard_publisher rclcpp std_msgs OpenCV)


# 설치 디렉터리 설정
install(TARGETS
  travi_ui_node
  keyboard_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
