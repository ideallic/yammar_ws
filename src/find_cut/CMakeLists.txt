#1---------------------------> Basic env. setting
cmake_minimum_required(VERSION 2.8.3)
project(find_cut)
add_definitions(-std=c++11)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  sensor_msgs
  roscpp
  rospy
  message_generation
  tf
  dosuss_srvs
)

#opencv
find_package( OpenCV REQUIRED )
#BOOST
set(BOOST_LIBS program_options serialization random system)
find_package(Boost 1.48.0 COMPONENTS ${BOOST_LIBS} REQUIRED)
MESSAGE("Boost information:") 
MESSAGE("  Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}") 
MESSAGE("  Boost_LIBRARIES: ${Boost_LIBRARIES}") 
MESSAGE("  Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")


catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
　${Boost_LIBRARY_DIRS}
 "/usr/local/lib"
)

#2--------------------------->  create library

#3--------------------------->  Create executeable file
add_executable(find_cut_node_realtime src/find_cut_node_realtime.cpp)
add_dependencies(find_cut_node_realtime dosuss_srvs_gencpp)
target_link_libraries(find_cut_node_realtime
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
 )

add_executable(get_image src/get_image.cpp)
add_dependencies(get_image dosuss_srvs_gencpp)
target_link_libraries(get_image
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
 )
