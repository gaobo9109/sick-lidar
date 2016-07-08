cmake_minimum_required(VERSION 2.8.3)
project(sick_lidar)

################################################################################
add_definitions( -DSICKLDMRS_EXPORTS )


# ========================================
# Find Packages
# ========================================
find_package(Qt4 REQUIRED)

if(QT4_FOUND)
  set(QT_USE_QTXML true)
  set(QT_USE_QTNETWORK true)
  include(${QT_USE_FILE})
else()
  message(ERROR "Qt4 needed")
endif()

find_package(PCL 1.3 REQUIRED COMPONENTS common io)

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
  LIBRARIES  ${PROJECT_NAME}
  CATKIN_DEPENDS nodelet roscpp rospy sensor_msgs std_msgs tf
  DEPENDS system_lib
)

# ========================================
# Compiler definitions
# ========================================
add_definitions(
  ${QT_DEFINITIONS}
  ${PCL_DEFINITIONS}
)

# ========================================
# Include directories
# ========================================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${QT_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# ========================================
# Link directories
# ========================================
link_directories(
  ${PCL_LIBRARY_DIRS}
)

# ========================================
# List of sources
# ========================================
set(PROJECT_SRCS
  src/SickLDMRSSensor.cpp
  src/SickSocket.cpp
  src/SickLDMRSROS.cpp
)



# ========================================
# Call MOC
# ========================================
qt4_wrap_cpp(PROJECT_MOC_SRCS
    include/SickLDMRSSensor.h
    include/SickSocket.h
)

# ========================================
# Build a library
# ========================================
add_library(
    ${PROJECT_NAME}
    ${PROJECT_SRCS}
    ${PROJECT_MOC_SRCS}
)

# ========================================
# Libraries
# ========================================
# All the platform
target_link_libraries(
    ${PROJECT_NAME}
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_COMMON_LIBRARIES} 
    ${PCL_IO_LIBRARIES}
)

