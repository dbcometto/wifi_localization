cmake_minimum_required(VERSION 3.5)
project(vn_interface)

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# declare messages and services to generate
set(msg_files
  "msg/Vectornav.msg"
)
set(srv_files
)

# generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
  sensor_msgs
)

ament_package()