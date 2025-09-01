cmake_minimum_required(VERSION 3.0.2)
project(connectivity_exploration)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

message("EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")

find_package( OpenCV )
if (OpenCV_FOUND)
  add_definitions ( -DOPENCV_LIBRARY_FOUND=12 )
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  message_filters
  visualization_msgs
  geometry_msgs
  message_generation
  tf2
  tf2_ros
  tf2_geometry_msgs
)


find_package(Boost REQUIRED COMPONENTS thread)


add_message_files(
  FILES
  PointArray.msg
)

# Dependence to generate self-defined messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES connectivity_exploration
    CATKIN_DEPENDS 
    roscpp
    rospy
    std_msgs
    tf
    sensor_msgs
    message_filters
    visualization_msgs
    geometry_msgs
    message_runtime
    tf2
    tf2_ros
    tf2_geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_filter src/scan_filter.cpp)
target_link_libraries(scan_filter ${catkin_LIBRARIES})

add_executable(pub_flipped_convexhull src/pub_flipped_convexhull.cpp)
target_link_libraries(pub_flipped_convexhull ${catkin_LIBRARIES} ${OpenCV_LIBS})

# add_executable(debug_scp src/debugSCP.cpp)
# target_link_libraries(debug_scp ${catkin_LIBRARIES} ${OpenCV_LIBS})

# add_executable(pubSCP src/pubSCP.cpp)
# target_link_libraries(pubSCP ${catkin_LIBRARIES} ${OpenCV_LIBS})