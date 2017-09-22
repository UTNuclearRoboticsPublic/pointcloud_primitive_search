cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_primitive_search)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  std_msgs 
  sensor_msgs
  visualization_msgs 
  message_generation
  pcl_conversions 
  pcl_ros
  pointcloud_processing_server
)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  DIRECTORY msg
  FILES 
  search_input.msg
  search_output.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  primitive_process.srv
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs visualization_msgs pointcloud_processing_server
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES reg_creation primitive_process_creation primitive_process_publisher
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${pcl_INCLUDE_DIRS})

add_executable(example_client src/example_client.cpp)
target_link_libraries( 
  example_client primitive_process_publisher primitive_process_creation
  ${catkin_LIBRARIES}
)
target_link_libraries(example_client)
add_dependencies(
  example_client 
  ${pointcloud_primitive_search_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(primitive_search src/primitive_search.cpp)
target_link_libraries( 
  primitive_search 
  ${catkin_LIBRARIES}
)
add_dependencies(
  primitive_search
  ${pointcloud_primitive_search_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

add_library(primitive_process_creation src/primitive_process_creation.cpp)
add_dependencies(primitive_process_creation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} pointcloud_utilities pointcloud_task_creation)

add_library(primitive_process_publisher src/primitive_process_publisher.cpp)
add_dependencies(primitive_process_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)