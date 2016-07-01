cmake_minimum_required(VERSION 2.8.3)
project(object_scanner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  pcl_conversions
  roscpp
  sensor_msgs
  std_msgs
  tf
)
find_package(FLANN REQUIRED)
find_package(PCL 1.7 REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_scanner
#  CATKIN_DEPENDS cmake_modules geometry_msgs libpcl-1.7-all pcl_conversions roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(SOURCE_FILES src/main.cpp src/object_scanner.cpp)
add_executable(object_scanner ${SOURCE_FILES})
target_link_libraries(object_scanner ${catkin_LIBRARIES}
				    ${PCL_LIBRARIES} )

