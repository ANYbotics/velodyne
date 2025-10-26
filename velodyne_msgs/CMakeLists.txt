cmake_minimum_required(VERSION 3.16.3)
project(any_velodyne_msgs)

if("$ENV{ROS_VERSION}" STREQUAL "1")

# Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    acl_fault_generator
    message_generation
    std_msgs
)

# Declare the service and message files to be built
add_message_files(
  DIRECTORY
    msg
  FILES
    VelodynePacket.msg
    VelodyneScan.msg
    VelodyneSyncStatus.msg
    VelodyneInterfaceStatus.msg
)
generate_messages(
  DEPENDENCIES
    std_msgs
)

# Declare faults
add_fault_definitions(
  FILES
    lidar.yaml
)
generate_faults()

# Catkin package macro
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)

else() # ROS version 2

set(PACKAGE_DEPENDENCIES
  builtin_interfaces
  acl_fault_generator
  std_msgs
  )

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

foreach(PKG IN LISTS PACKAGE_DEPENDENCIES)
  find_package(${PKG} REQUIRED)
endforeach()

set(MSG_FILES
  "msg_ros2/VelodynePacket.msg"
  "msg/VelodyneScan.msg"
  "msg_ros2/VelodyneSyncStatus.msg"
  "msg/VelodyneInterfaceStatus.msg"
  )

rosidl_generate_interfaces(${PROJECT_NAME}
    ${MSG_FILES}
  DEPENDENCIES
    ${PACKAGE_DEPENDENCIES}
  )

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

add_fault_definitions(FILES lidar.yaml)
generate_faults()

ament_package()

endif()
