cmake_minimum_required(VERSION 3.0.2)

project(fkie_measurement_server)

find_package(
  catkin REQUIRED
  COMPONENTS 
    roscpp
    rospy
    std_msgs
    tf
    tf2_ros
    fkie_measurement_msgs
    actionlib_msgs
    geometry_msgs
  )

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    fkie_measurement_msgs
    actionlib_msgs
    geometry_msgs
    tf
)

# ##########
# Install ##
# ##########

catkin_install_python(
  PROGRAMS 
    nodes/measurement_server_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
