cmake_minimum_required(VERSION 2.8.3)
project(termica_reconstrucao)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(catkin REQUIRED cmake_modules COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_msgs
)

set(${PROJECT_NAME}_CATKIN_COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
  message_filters
  dynamic_reconfigure
  stereo_msgs
  tf
  camera_calibration_parsers
  camera_info_manager
  image_geometry
 # highgui
)

find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_COMPONENTS}
)

find_package(OpenCV)


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES termica_reconstrucao
  CATKIN_DEPENDS std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(grabber src/grabber.cpp)
add_dependencies(grabber ${catkin_EXPORTED_TARGETS})

add_executable(reconstrucao_termica ${PROJECT_SOURCE_DIR}/src/reconstrucao_termica.cpp)
add_dependencies(reconstrucao_termica ${catkin_EXPORTED_TARGETS})

add_executable(reconstrucao ${PROJECT_SOURCE_DIR}/src/reconstrucao.cpp)
add_dependencies(reconstrucao ${catkin_EXPORTED_TARGETS})

add_executable(termica_cor ${PROJECT_SOURCE_DIR}/src/termica_cor.cpp)
add_dependencies(termica_cor ${catkin_EXPORTED_TARGETS})

add_executable(reconstrucao_online ${PROJECT_SOURCE_DIR}/src/reconstrucao_online.cpp)
add_dependencies(reconstrucao_online ${catkin_EXPORTED_TARGETS})

add_executable(reconstrucaoF ${PROJECT_SOURCE_DIR}/src/reconstrucaoF.cpp)
add_dependencies(reconstrucaoF ${catkin_EXPORTED_TARGETS})

target_link_libraries(grabber
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Eigen_LIBRARIES}
)

target_link_libraries(reconstrucao_termica
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Eigen_LIBRARIES}
)

target_link_libraries(reconstrucao
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Eigen_LIBRARIES}
)

target_link_libraries(termica_cor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Eigen_LIBRARIES}
)

target_link_libraries(reconstrucao_online
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Eigen_LIBRARIES}
)

target_link_libraries(reconstrucaoF
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Eigen_LIBRARIES}
)
