cmake_minimum_required(VERSION 3.0.2)
project(count)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  roscpp
  std_msgs
  sensor_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  rospy
  message_generation
  octomap_msgs
  octomap_server
  octomap_rviz_plugins
)
find_package(Eigen3 REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} )


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)



catkin_package()


add_executable(voxelcount src/octomapcount.cpp)
target_link_libraries(voxelcount ${catkin_LIBRARIES})
#add_dependencies(control arm_controller)
add_dependencies(voxelcount ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})





