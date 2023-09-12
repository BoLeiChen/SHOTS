cmake_minimum_required(VERSION 3.0.2)

project(fkie_measurement_tools)

add_definitions(-DNDEBUG) # supress Octomap debug output

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  octomap_msgs
  octomap_ros
  geometry_msgs
  nav_msgs
  visualization_msgs
  pcl_ros
  fkie_measurement_msgs
  actionlib
  tf2
  tf2_geometry_msgs
  grid_map_core
  grid_map_ros
  fkie_ddynamic_reconfigure
)

find_package(octomap REQUIRED)

# Add OpenMP 
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp
    std_msgs
    octomap_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    fkie_measurement_msgs
    actionlib
    tf2
    tf2_geometry_msgs
    fkie_ddynamic_reconfigure
)

###########
## Build ##
###########

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

# occupancy_grid_to_point_cloud node ---------------------------------------------------------------------------------
add_executable(occupancy_grid_to_point_cloud 
  src/occupancy_grid_to_point_cloud.cpp
  src/OccupancyGridToPointCloud.cpp
)
target_link_libraries(occupancy_grid_to_point_cloud ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} -ltbb)
add_dependencies(occupancy_grid_to_point_cloud ${${PROJECT_NAME}_generate_messages_cpp}
                  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 
# measurements_to_point_cloud node ---------------------------------------------------------------------------------
add_executable(measurements_to_point_cloud 
  src/measurements_to_point_cloud.cpp
  src/MeasurementsToPointCloud.cpp
)
target_link_libraries(measurements_to_point_cloud ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} -ltbb)
add_dependencies(measurements_to_point_cloud ${${PROJECT_NAME}_generate_messages_cpp}
                  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS 
    occupancy_grid_to_point_cloud
    measurements_to_point_cloud
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)