cmake_minimum_required(VERSION 3.0.2)

project(fkie_nbv_planner)

# SET OCTOMAP_OMP to disable/enable OPENMP (experimental for octomap, default disabled)
SET(OCTOMAP_OMP FALSE CACHE BOOL "Enable/disable OpenMP parallelization")
IF(DEFINED ENV{OCTOMAP_OMP})
  SET(OCTOMAP_OMP $ENV{OCTOMAP_OMP})
ENDIF(DEFINED ENV{OCTOMAP_OMP})
IF(OCTOMAP_OMP)
  FIND_PACKAGE( OpenMP REQUIRED)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
ENDIF(OCTOMAP_OMP)

find_package(PCL REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  gps_common
  jsk_rviz_plugins
  mbf_msgs
  message_generation
  message_runtime
  nav_msgs
  octomap_msgs
  octomap_server
  octomap_rviz_plugins
  roscpp
  std_msgs
  tf
  tf2
  tf2_geometry_msgs
  visualization_msgs
  fkie_measurement_msgs
  fkie_ddynamic_reconfigure
  sensor_msgs
  pcl_ros
  pcl_conversions
  std_srvs
)

find_package(octomap REQUIRED)
find_package(CGAL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN3 REQUIRED eigen3)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_action_files(
  DIRECTORY action 
  FILES 
  NbvPlanner.action 
)

add_service_files(
  FILES 
  SetObjPose.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  fkie_measurement_msgs
  geometry_msgs
  mbf_msgs
  nav_msgs
  std_msgs
  visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS octomap PCL

  LIBRARIES 
  fkie_nbv_planner 
  ${catkin_LIBRARIES} 
  ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  geometry_msgs
  gps_common
  mbf_msgs
  message_generation
  message_runtime
  octomap_msgs
  roscpp
  std_msgs
  nav_msgs
  tf
  tf2
  tf2_geometry_msgs
  visualization_msgs
  fkie_measurement_msgs  
  fkie_ddynamic_reconfigure
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(fkie_nbv_planner 
  src/NBVPlanner.cpp 
  src/NBVRviz.cpp
  src/RRTNode.cpp
  src/PathOptimizer.cpp
  src/NBVParameters.cpp
)

target_link_libraries(fkie_nbv_planner
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  CGAL gmp
)

add_dependencies(fkie_nbv_planner
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
)

add_executable(nbvplanner_node 
  src/nbvplanner_node.cpp 
  src/NBVPlanner.cpp 
  src/NBVRviz.cpp
  src/RRTNode.cpp
  src/PathOptimizer.cpp
  src/NBVParameters.cpp
  src/octomap_generator/octomap_generator.cpp
  src/semantic_octree/semantics_bayesian.cpp
  src/semantic_octree/semantics_max.cpp
)


add_executable(octomap_generator
 src/octomap_generator/octomap_generator.cpp
 src/octomap_generator/octomap_generator_ros.cpp
 src/semantic_octree/semantics_bayesian.cpp
 src/semantic_octree/semantics_max.cpp
)

add_executable(octomap_test
 src/octomap_generator/octomap_test.cpp
)

target_link_libraries(nbvplanner_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  CGAL gmp
)

target_link_libraries(octomap_generator ${LINK_LIBS})
target_link_libraries(octomap_test ${LINK_LIBS})

add_dependencies(nbvplanner_node 
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
)


#############
## Install ##
#############

install(TARGETS 
  nbvplanner_node
  fkie_nbv_planner
  EXPORT fkie_nbv_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PUBLIC_HEADER DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


install(PROGRAMS
  scripts/odom2Posestamped.py
  scripts/camera_pose_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)






