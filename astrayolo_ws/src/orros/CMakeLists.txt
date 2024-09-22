cmake_minimum_required(VERSION 3.5)
project(orros)

# 메시지 생성 의존성
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 메시지 파일 추가
set(msg_files
  "msg/YoloBox.msg"
  "msg/YoloBoxes.msg"
)

# 메시지 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

# 의존성 내보내기
ament_export_dependencies(rosidl_default_runtime)
ament_package()

