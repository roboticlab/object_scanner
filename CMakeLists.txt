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

#compute flags macros
MACRO(CUDA_COMPUTE_TARGET_FLAGS arch_bin arch_ptx cuda_nvcc_target_flags)
    string(REGEX REPLACE "\\." "" ARCH_BIN_WITHOUT_DOTS "${${arch_bin}}")
    string(REGEX REPLACE "\\." "" ARCH_PTX_WITHOUT_DOTS "${${arch_ptx}}")
                                
    set(cuda_computer_target_flags_temp "") 
    
    # Tell NVCC to add binaries for the specified GPUs
    string(REGEX MATCHALL "[0-9()]+" ARCH_LIST "${ARCH_BIN_WITHOUT_DOTS}")
    foreach(ARCH IN LISTS ARCH_LIST)
        if (ARCH MATCHES "([0-9]+)\\(([0-9]+)\\)")
            # User explicitly specified PTX for the concrete BIN                    
            set(cuda_computer_target_flags_temp ${cuda_computer_target_flags_temp} -gencode arch=compute_${CMAKE_MATCH_2},code=sm_${CMAKE_MATCH_1})                 
        else()                  
            # User didn't explicitly specify PTX for the concrete BIN, we assume PTX=BIN                                
            set(cuda_computer_target_flags_temp ${cuda_computer_target_flags_temp} -gencode arch=compute_${ARCH},code=sm_${ARCH})                   
        endif()
    endforeach()
                
    # Tell NVCC to add PTX intermediate code for the specified architectures
    string(REGEX MATCHALL "[0-9]+" ARCH_LIST "${ARCH_PTX_WITHOUT_DOTS}")
    foreach(ARCH IN LISTS ARCH_LIST)                
        set(cuda_computer_target_flags_temp ${cuda_computer_target_flags_temp} -gencode arch=compute_${ARCH},code=compute_${ARCH})              
    endforeach()    
                            
    set(${cuda_nvcc_target_flags} ${cuda_computer_target_flags_temp})       
ENDMACRO()

MACRO(APPEND_TARGET_ARCH_FLAGS)
    set(cuda_nvcc_target_flags "")
    CUDA_COMPUTE_TARGET_FLAGS(CUDA_ARCH_BIN CUDA_ARCH_PTX cuda_nvcc_target_flags)       
    if (cuda_nvcc_target_flags)
        message(STATUS "CUDA NVCC target flags: ${cuda_nvcc_target_flags}")
        list(APPEND CUDA_NVCC_FLAGS ${cuda_nvcc_target_flags})
    endif()
ENDMACRO()

find_package(CUDA REQUIRED)
if(${CUDA_VERSION_STRING} VERSION_GREATER "4.1")
        set(CUDA_ARCH_BIN "2.0 2.1(2.0) 3.0 5.0 5.2" CACHE STRING "Specify 'real' GPU architectures to build binaries for, BIN(PTX) format is supported")
    else()
        set(CUDA_ARCH_BIN "2.0 2.1(2.0)" CACHE STRING "Specify 'real' GPU architectures to build binaries for, BIN(PTX) format is supported")
    endif()
APPEND_TARGET_ARCH_FLAGS()
message(STATUS "CUDA_BIN_ARCH ${CUDA_ARCH_BIN}")

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
  ${CUDA_INCLUDE_DIRS}
#   /home/elena/sources/pcl-pcl-1.8.0/gpu/features/include
#   /home/elena/sources/pcl-pcl-1.8.0/gpu/utils/include
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


