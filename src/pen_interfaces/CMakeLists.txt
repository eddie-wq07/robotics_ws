cmake_minimum_required(VERSION 3.8)
project(pen_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteDrawing.action" "action/PrepareDrawing.action" "action/GeneratePath.action" "msg/PenEvent.msg")
ament_export_dependencies(rosidl_default_runtime)
ament_package()
