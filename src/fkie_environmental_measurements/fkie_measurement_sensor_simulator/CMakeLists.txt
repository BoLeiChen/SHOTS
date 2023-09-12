cmake_minimum_required(VERSION 3.0.2)
project(fkie_measurement_sensor_simulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  fkie_measurement_msgs
  tf2_ros
  tf
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    roscpp
    tf
    tf2_ros
    fkie_measurement_msgs
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(SensorSimulatorNode 
  src/SensorSimulator.cpp
  src/SensorSimulatorNode.cpp)

target_link_libraries(SensorSimulatorNode ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES} -ltbb)
                      
add_dependencies(SensorSimulatorNode ${${PROJECT_NAME}_generate_messages_cpp}
                 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
  TARGETS 
    SensorSimulatorNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
