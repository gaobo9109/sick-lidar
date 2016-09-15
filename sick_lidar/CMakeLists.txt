cmake_minimum_required(VERSION 2.8.3)
project(sick_lidar)

find_package(Qt4 REQUIRED)

if(QT4_FOUND)
  set(QT_USE_QTXML true)
  set(QT_USE_QTNETWORK true)
  include(${QT_USE_FILE})
else()
  message(ERROR "Qt4 needed")
endif()

find_package(PCL 1.3 REQUIRED COMPONENTS)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nodelet
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  dynamic_reconfigure
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sick_lib
  CATKIN_DEPENDS nodelet roscpp rospy sensor_msgs std_msgs tf
  DEPENDS system_lib
)

add_definitions(
  ${QT_DEFINITIONS}
  ${PCL_DEFINITIONS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${QT_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

qt4_wrap_cpp(PROJECT_MOC_SRCS
    include/SickLDMRSSensor.h
    include/SickSocket.h
)

add_library(sick_lib
    src/SickLDMRSROS.cpp
    src/SickLDMRSCommand.cpp
    ${PROJECT_MOC_SRCS}
)


target_link_libraries(sick_lib
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_COMMON_LIBRARIES} 
    ${PCL_IO_LIBRARIES}
)

add_executable(driver
    src/SickDriver.cpp
    src/SickSocket.cpp
    src/SickLDMRSSensor.cpp
)

target_link_libraries(driver
    sick_lib
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(pcl_cluster src/pcl_clustering.cpp)
target_link_libraries(pcl_cluster 
    ${catkin_LIBRARIES}    
    ${PCL_LIBRARIES})

