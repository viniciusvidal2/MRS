cmake_minimum_required(VERSION 2.8.3)
project(projecao_overlap)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
    tf_conversions
    pcl_ros
    cv_bridge
    image_transport
    geometry_msgs
    camera_calibration_parsers
    camera_info_manager
    image_geometry)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES projecao_overlap
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

add_executable(projecao_overlap_node src/projecao_overlap.cpp)
target_link_libraries (projecao_overlap_node ${PCL_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(sincroniza_odometrias src/sincroniza_odometrias.cpp)
target_link_libraries (sincroniza_odometrias ${PCL_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(salvaDadosZedNode src/dadosComZed.cpp)
target_link_libraries (salvaDadosZedNode ${OpenCV_LIBS} ${Eigen_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES} )
