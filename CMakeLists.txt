cmake_minimum_required(VERSION 2.8.3)
project(object_scanner)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++0x") # Need C++11 
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs  
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  pcl_conversions
  moveit_core
  moveit_commander
  moveit_msgs
  moveit_ros_planning_interface
)
find_package(Eigen REQUIRED)
find_package(FLANN REQUIRED)
find_package(PCL 1.7 REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_scanner
#  CATKIN_DEPENDS cmake_modules geometry_msgs libpcl-1.7-all pcl_conversions roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(SYSTEM
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR} 
  ${EIGEN_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# add_library(cloud_processor src/object_scanner.cpp src/robots_mover.cpp src/robot_mover.cpp src/tsdf.cpp  src/cloud_processor.cpp)
# message("\n \n \n \n" ${PCL_LIBRARIES})
# link_libraries(cloud_processor ${PCL_LIBRARIES})
# 
# set(SOURCE_FILES src/main.cpp src/object_scanner.cpp src/robots_mover.cpp src/robot_mover.cpp src/tsdf.cpp src/cloud_processor.cpp)
add_executable(object_scanner src/main.cpp src/object_scanner.cpp src/robots_mover.cpp src/robot_mover.cpp src/tsdf.cpp src/cloud_processor.cpp)
target_link_libraries(object_scanner  ${catkin_LIBRARIES}
				    ${PCL_LIBRARIES} 
				    ${EIGEN_LIBRARIES} 
				    ${Boost_LIBRARIES}
# 				    cloud_processor
)

