cmake_minimum_required(VERSION 3.8)
project(sbim_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/PrincipalDirectionArray.msg"
  "msg/PrincipalPlaneArray.msg"
  "msg/PrincipalPlane.msg"
  "msg/CorrespondenceMap.msg"
  "msg/Door.msg"
  "msg/DoorArray.msg"
  "msg/LayoutSegment.msg"
  "msg/LayoutSegmentArray.msg"
  "msg/Trajectory.msg"
  "msg/SceneNode.msg"
  "msg/SceneEdge.msg"
  "msg/Floorplan.msg"
  "msg/FloorplanArray.msg"
  "msg/UInt32Pair.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs sensor_msgs shape_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

