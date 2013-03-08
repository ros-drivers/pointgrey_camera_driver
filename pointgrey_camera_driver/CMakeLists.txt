cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Find the flycapture2 library from Point Grey
include_directories("/usr/include/flycapture")
find_library(FLYCAPTURE2 flycapture)

if(FLYCAPTURE2)
  # Link flycapture to pointgrey
  rosbuild_add_library(PointGreyCamera src/PointGreyCamera.cpp)
  target_link_libraries(PointGreyCamera ${FLYCAPTURE2})

  # Link pointgrey to the nodelets
  rosbuild_add_library(PointGreyCameraNodelet src/nodelet.cpp)
  target_link_libraries(PointGreyCameraNodelet PointGreyCamera)

  rosbuild_add_library(PointGreyStereoCameraNodelet src/stereo_nodelet.cpp)
  target_link_libraries(PointGreyStereoCameraNodelet PointGreyCamera)

  # Make the standalone nodes
  rosbuild_add_executable(pointgrey_camera_node src/node.cpp)

  rosbuild_add_executable(pointgrey_stereo_node src/stereo_node.cpp)

  # Make the list cameras tool
  rosbuild_add_executable(list_cameras src/list_cameras.cpp)
  target_link_libraries(list_cameras PointGreyCamera)
  
else(FLYCAPTURE2)
  message(FATAL_ERROR "FlyCapture2 is needed for the Point Grey cameras.  Cannot continue.")
endif(FLYCAPTURE2)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
