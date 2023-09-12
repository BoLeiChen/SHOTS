cmake_minimum_required(VERSION 3.0.2)
project(fkie_ddynamic_reconfigure)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options("-Wno-pedantic")
add_compile_options("-Wno-old-style-cast")
add_compile_options("-Wno-redundant-decls")
add_compile_options("-Wno-ctor-dtor-privacy")
add_compile_options("-Wno-non-virtual-dtor")


find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp dynamic_reconfigure
)

include_directories(
  include
)
include_directories(
  SYSTEM 
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/ddynamic_reconfigure.cpp src/GlobalParameters.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
#add_executable(dynamic_reconfigure_node src/dynamic_reconfigure_node.cpp)
#target_link_libraries(dynamic_reconfigure_node ${catkin_LIBRARIES})

#make sure configure headers are built before any node using them
#add_dependencies(dynamic_reconfigure_node ${PROJECT_NAME}_gencfg)

#############  Test #################
add_executable(fake_dynamic_reconfigure_server test/fake_dynamic_reconfigure_server.cpp )
target_link_libraries(fake_dynamic_reconfigure_server ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(test_bool_dynamic_reconfigure_server test/test_bool_dynamic_reconfigure_server.cpp)
target_link_libraries(test_bool_dynamic_reconfigure_server ${PROJECT_NAME} ${catkin_LIBRARIES})

# add_executable(test_GlobalParameters test/test_GlobalParameters.cpp)
# target_link_libraries(test_GlobalParameters ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gmock(fkie_ddynamic_reconfigure-test test/ddynamic_reconfigure.test test/test_ddynamic_reconfigure.cpp)
    target_link_libraries(fkie_ddynamic_reconfigure-test ${PROJECT_NAME})
    add_executable(fkie_ddynamic_reconfigure_auto_update_test test/test_bool_dynamic_reconfigure_server.cpp)
    target_link_libraries(fkie_ddynamic_reconfigure_auto_update_test ${PROJECT_NAME} ${catkin_LIBRARIES})

    # add_rostest_gmock(fkie_GlobalParameters-test test/GlobalParameters.test test/test_GlobalParameters.cpp)
    # target_link_libraries(fkie_GlobalParameters-test ${PROJECT_NAME})
    # add_executable(fkie_GlobalParameters_test test/test_GlobalParameters.cpp)
    # target_link_libraries(fkie_GlobalParameters_test ${PROJECT_NAME} ${catkin_LIBRARIES})

    add_rostest_gtest(fkie_GlobalParameters-test test/GlobalParameters.test test/test_GlobalParameters.cpp src/GlobalParameters.cpp)
    target_link_libraries(fkie_GlobalParameters-test ${PROJECT_NAME} ${catkin_LIBRARIES})

endif(CATKIN_ENABLE_TESTING)
