cmake_minimum_required(VERSION 3.16.3)
project(any_velodyne_msgs)

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
