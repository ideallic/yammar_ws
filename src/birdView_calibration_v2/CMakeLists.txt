#1--------------------------->
cmake_minimum_required(VERSION 3.2)
project(birdView_calibration_v2)
add_definitions(-std=c++11)                                          #specify compiler

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

#eigen3
set(EIGEN_I /usr/include/eigen3)
#OpenCV
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
  ${EIGEN_I} 
  ${Boost_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

link_directories(
  ${Boost_LIBRARY_DIRS}
  "/usr/local/lib"
  ${PROJECT_SOURCE_DIR}/lib
)

#2---------------------------> create library

#add_library(RBTREE ${SOURCE_PATH}/Rbtree.cpp)
#add_library(CircleList ${SOURCE_PATH}/CircleLinkedList.cpp)
#add_library(Common_lib ${SOURCE_PATH}/Common.cpp)
#add_library(json_lib ${SOURCE_PATH}/jsoncpp.cpp)
#Sources files
set(birdView_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/DosuBirdView.cpp
#  ${PROJECT_SOURCE_DIR}/src/Rbtree.cpp
#  ${PROJECT_SOURCE_DIR}/src/CircleLinkedList.cpp
#  ${PROJECT_SOURCE_DIR}/src/jsoncpp.cpp
#  ${PROJECT_SOURCE_DIR}/src/FrameShell.cpp
#  ${PROJECT_SOURCE_DIR}/src/Common.cpp
)

add_library(DosuBird_lib ${birdView_SOURCE_FILES})
add_dependencies(DosuBird_lib dosuss_srvs_gencpp)
#target_link_libraries(DosuRay_lib
#		      ${catkin_LIBRARIES} 
#		      ${OpenCV_LIBS}
#                      ${Boost_LIBRARIES})

#3--------------------------->  Create executeable file
add_executable(birdView_calibration_v2 ${PROJECT_SOURCE_DIR}/src/main.cpp)
#use DosuBird_lib, so don't need the following line
#add_dependencies(birdView_calibration_v2 dosuss_srvs_gencpp)

target_link_libraries(birdView_calibration_v2
  DosuBird_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
)

