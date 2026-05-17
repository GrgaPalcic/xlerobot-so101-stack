cmake_minimum_required(VERSION 3.5)
project(so101_grasp_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraspCandidate.msg"
  "srv/DetectGrasps.srv"
  "srv/PlanGrasp.srv"
  DEPENDENCIES geometry_msgs std_msgs trajectory_msgs
)

ament_package()
