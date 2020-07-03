cmake_minimum_required(VERSION 2.8.1)
project(any_velodyne_driver)

set(CMAKE_CXX_STANDARD 11)

set(${PROJECT_NAME}_CATKIN_DEPS 
    diagnostic_updater
    dynamic_reconfigure
    nodelet
    roscpp
    tf
    any_velodyne_msgs
)

find_package(catkin REQUIRED
  COMPONENTS
    ${${PROJECT_NAME}_CATKIN_DEPS}
    roslint
)
find_package(Boost REQUIRED COMPONENTS thread)

# libpcap provides no pkg-config or find_package module:
set(libpcap_LIBRARIES -lpcap)

include_directories(
  include 
  SYSTEM
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

# Generate dynamic_reconfigure server
generate_dynamic_reconfigure_options(cfg/VelodyneNode.cfg)

# objects needed by other ROS packages that depend on this one
catkin_package(
  CATKIN_DEPENDS
    ${${PROJECT_NAME}_CATKIN_DEPS}
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_socket
    ${PROJECT_NAME}
    ${PROJECT_NAME}_nodelet
)

#############
# Libraries #
#############
# Socket library.
add_library(${PROJECT_NAME}_socket 
  src/lib/input.cc
)
add_dependencies(${PROJECT_NAME}_socket
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_socket
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# ROS wrapper.
add_library(${PROJECT_NAME}
  src/driver/driver.cc
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_socket
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

##########################
##  Nodes and nodelets  ##
##########################
# Node.
add_executable(${PROJECT_NAME}_node
  src/velodyne_driver_node.cc
)
add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_socket
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# Nodelet.
add_library(${PROJECT_NAME}_nodelet
  src/velodyne_driver_nodelet.cc
)
add_dependencies(${PROJECT_NAME}_nodelet
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(${PROJECT_NAME}_nodelet
  ${PROJECT_NAME}_socket
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

###########
# Install #
###########
install(
  TARGETS
    ${PROJECT_NAME}_socket
    ${PROJECT_NAME}
    ${PROJECT_NAME}_node
    ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  FILES
    nodelet_velodyne.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
  DIRECTORY
    launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
  PROGRAMS
    src/vdump
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# roslint_cpp()

# if (CATKIN_ENABLE_TESTING)

#   # these dependencies are only needed for unit testing
#   find_package(roslaunch REQUIRED)
#   find_package(rostest REQUIRED)

#   # Download packet capture (PCAP) files containing test data.
#   # Store them in devel-space, so rostest can easily find them.
#   catkin_download_test_data(
#     ${PROJECT_NAME}_tests_class.pcap
#     http://download.ros.org/data/velodyne/class.pcap
#     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
#     MD5 65808d25772101358a3719b451b3d015)
#   catkin_download_test_data(
#     ${PROJECT_NAME}_tests_32e.pcap
#     http://download.ros.org/data/velodyne/32e.pcap
#     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
#     MD5 e41d02aac34f0967c03a5597e1d554a9)
#   catkin_download_test_data(
#     ${PROJECT_NAME}_tests_vlp16.pcap
#     http://download.ros.org/data/velodyne/vlp16.pcap
#     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
#     MD5 f45c2bb1d7ee358274e423ea3b66fd73)
  
#   # unit tests
#   add_rostest(tests/pcap_node_hertz.test)
#   add_rostest(tests/pcap_nodelet_hertz.test)
#   add_rostest(tests/pcap_32e_node_hertz.test)
#   add_rostest(tests/pcap_32e_nodelet_hertz.test)
#   add_rostest(tests/pcap_vlp16_node_hertz.test)
#   add_rostest(tests/pcap_vlp16_nodelet_hertz.test)
  
#   # parse check all the launch/*.launch files
#   roslaunch_add_file_check(launch)

#   # unit test
#   catkin_add_gtest(time_test tests/timeconversiontest.cpp)
#   target_link_libraries(time_test
#     ${catkin_LIBRARIES}
#     ${Boost_LIBRARIES})
# endif (CATKIN_ENABLE_TESTING)
