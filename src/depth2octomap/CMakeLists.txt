cmake_minimum_required(VERSION 2.8.3)
project(depth2octomap)
 
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        rostime
        sensor_msgs
        message_filters
        cv_bridge
        image_transport
)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)
 
catkin_package()
 
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
 
add_executable (depth2octomap src/depth2octomap.cpp)
target_link_libraries (depth2octomap ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
