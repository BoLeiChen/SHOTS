cmake_minimum_required(VERSION 3.0.2)

project(fkie_measurement_msgs)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             rospy
             std_msgs
             message_generation
             message_runtime
             actionlib_msgs
             geometry_msgs)

add_action_files(
  DIRECTORY action
  FILES 
    RequestBoundaryPolygons.action 
    RequestGaussianInterpolation.action
    ExplorePolygon3D.action
)

add_message_files(
  FILES 
    Measurement.msg
    MeasurementValue.msg
    MeasurementLocated.msg
    MeasurementArray.msg
    MeasurementGaussianStatistics.msg
    BoundaryPolygon.msg 
    MeasurementEstimation.msg
    MeasurementEstimationValue.msg)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    message_runtime
    actionlib_msgs
    geometry_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)