cmake_minimum_required(VERSION 3.8)
project(task_manager)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # <-- 추가
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "../../deli_interfaces/task_manager_msgs/action/DispatchDeliveryTask.action"
  #"../../deli_interfaces/task_manager_msgs/action/DispatchManipulationTask.action"
  "../../deli_interfaces/task_manager_msgs/msg/BatteryLevel.msg"
  "../../deli_interfaces/task_manager_msgs/msg/Payload.msg"
  "../../deli_interfaces/task_manager_msgs/msg/PickUp.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)


ament_python_install_package(  # <-- 파이썬 패키지 설치
  task_manager_py
  PACKAGE_DIR task_manager_py
)

install(
  PROGRAMS
    task_manager_py/main.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME main
)

install(
  FILES domain_bridge.yaml
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
