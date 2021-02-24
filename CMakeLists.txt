cmake_minimum_required(VERSION 2.8.3)
project(mood_ros)

# Compile as C++11, supported in ROS Kinetic and newer add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin REQUIRED
# COMPONENTS xyz) is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure pluginlib uav_ros_lib
                                        message_filters image_transport)

# System dependencies are found with CMake's conventions find_package(Boost REQUIRED
# COMPONENTS system)

# Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(cfg/BlobDetectorParams.cfg)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

# ########################################################################################
# catkin specific configuration ##
# ########################################################################################
# The catkin_package macro generates cmake config files for your package Declare things to
# be passed to dependent projects INCLUDE_DIRS: uncomment this if your package contains
# header files LIBRARIES: libraries you create in this project that dependent projects
# also need CATKIN_DEPENDS: catkin_packages dependent projects also need DEPENDS: system
# dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  blob_detector
  single_msg_sync
  double_msg_sync
  CATKIN_DEPENDS
  uav_ros_lib
  DEPENDS
  OpenCV
  PCL)

# ########################################################################################
# Build ##
# ########################################################################################

# Specify additional locations of header files Your package locations should be listed
# before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

add_library(blob_detector src/blob_detector.cpp)
add_dependencies(blob_detector ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(blob_detector ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_library(single_msg_sync src/single_msg_sync.cpp)
add_dependencies(single_msg_sync ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(single_msg_sync ${catkin_LIBRARIES})

add_library(double_msg_sync src/double_msg_sync.cpp)
add_dependencies(double_msg_sync ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(double_msg_sync ${catkin_LIBRARIES})

add_executable(detection_manager_node nodes/detection_manager.cpp)
add_dependencies(detection_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(detection_manager_node ${catkin_LIBRARIES})

# Rename C++ executable without prefix The above recommended prefix causes long target
# names, the following renames the target back to the shorter version for ease of user use
# e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# Specify libraries to link a library or executable target against

# ########################################################################################
# Install ##
# ########################################################################################

# Mark executables and/or libraries for installation

install(
  TARGETS blob_detector single_msg_sync double_msg_sync
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# ########################################################################################
# Testing ##
# ########################################################################################

# Add gtest based cpp test target and link libraries catkin_add_gtest(${PROJECT_NAME}-test
# test/test_mood_ros.cpp) if(TARGET ${PROJECT_NAME}-test)
# target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}) endif()

# Add folders to be run by python nosetests catkin_add_nosetests(test)
