cmake_minimum_required(VERSION 2.8.3)
project(pointgrey_camera_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp nodelet sensor_msgs wfov_camera_msgs
  image_exposure_msgs camera_info_manager image_transport
  dynamic_reconfigure diagnostic_updater)

generate_dynamic_reconfigure_options(
  cfg/PointGrey.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp nodelet
)

# If flycapture is already present, use the found version. If not, download it.
# We can't resolve this dependency using the usual rosdep means because
# the Point Grey EULA prohibits redistributing the headers or the packages which
# contains them. We work around this by downloading the archive directly from
# their website during this step in the build process.
find_library(POINTGREY_LIB NAMES libflycapture.so.2 flycapture)
if(NOT POINTGREY_LIB)
  # flycapture not present, must download.
  message(STATUS "libflycapture not found in system library path")
  include(cmake/DownloadFlyCap.cmake)
  download_flycap(POINTGREY_LIB POINTGREY_INCLUDE_DIR)
  message(STATUS "libflycapture library: ${POINTGREY_LIB}")
  message(STATUS "libflycapture include: ${POINTGREY_INCLUDE_DIR}")
else()
  string(FIND "${POINTGREY_LIB}" "${CATKIN_DEVEL_PREFIX}" SUBSTR_INDEX)
  if(${SUBSTR_INDEX} EQUAL 0)
    # found flycapture, but it's already in our build space; we're reconfiguring.
    message(STATUS "libflycapture found in: ${POINTGREY_LIB}")
    set(POINTGREY_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/usr/include")
  else()
    # copy it into the build directory/resolving all symlinks
    message(STATUS "libflycapture found in system library path")
    message(STATUS "libflycapture library: ${POINTGREY_LIB}")

    get_filename_component(REAL_FLYCAPTURE ${POINTGREY_LIB} REALPATH)
    get_filename_component(POINTGREY_LIB ${POINTGREY_LIB} NAME)
    set(POINTGREY_LIB "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/${POINTGREY_LIB}")

    configure_file(${REAL_FLYCAPTURE} ${POINTGREY_LIB} COPYONLY)
    message(STATUS "libflycapture copied from: ${REAL_FLYCAPTURE} into ${POINTGREY_LIB}")
  endif()
endif()

include_directories(include ${POINTGREY_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(PointGreyCamera src/PointGreyCamera.cpp)
target_link_libraries(PointGreyCamera ${POINTGREY_LIB} ${catkin_LIBRARIES})
add_dependencies(PointGreyCamera ${PROJECT_NAME}_gencfg)

add_library(PointGreyCameraNodelet src/nodelet.cpp)
target_link_libraries(PointGreyCameraNodelet PointGreyCamera ${catkin_LIBRARIES})

add_library(PointGreyStereoCameraNodelet src/stereo_nodelet.cpp)
target_link_libraries(PointGreyStereoCameraNodelet PointGreyCamera ${catkin_LIBRARIES})

add_executable(pointgrey_camera_node src/node.cpp)
target_link_libraries(pointgrey_camera_node PointGreyCamera ${catkin_LIBRARIES})
set_target_properties(pointgrey_camera_node
                      PROPERTIES OUTPUT_NAME camera_node PREFIX "")

add_executable(pointgrey_stereo_node src/stereo_node.cpp)
target_link_libraries(pointgrey_stereo_node PointGreyCamera ${catkin_LIBRARIES})
set_target_properties(pointgrey_stereo_node
                      PROPERTIES OUTPUT_NAME stereo_node PREFIX "")

add_executable(pointgrey_list_cameras src/list_cameras.cpp)
target_link_libraries(pointgrey_list_cameras PointGreyCamera ${catkin_LIBRARIES})
set_target_properties(pointgrey_list_cameras
                      PROPERTIES OUTPUT_NAME list_cameras PREFIX "")

install(TARGETS
  PointGreyCamera
  PointGreyCameraNodelet
  PointGreyStereoCameraNodelet
  pointgrey_camera_node
  pointgrey_stereo_node
  pointgrey_list_cameras
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Redistributing the flycapture .so file is permitted by the SDK EULA:
# http://www.ptgrey.com/support/kb/data/PGR-FlyCap-SDK-LA.pdf
install(FILES ${POINTGREY_LIB} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_BUILD_BINARY_PACKAGE)
  # Install udev rules with dh_installudev: http://manpages.ubuntu.com/manpages/man1/dh_installudev.1.html
  string(REPLACE "_" "-" DEBIAN_PACKAGE_NAME ros-$ENV{ROS_DISTRO}-${PROJECT_NAME})
  file(REMOVE ${CMAKE_CURRENT_SOURCE_DIR}/debian/${DEBIAN_PACKAGE_NAME}.udev)
  file(GLOB UDEV_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/udev" "${CMAKE_CURRENT_SOURCE_DIR}/udev/*.rules")
  foreach(FILENAME ${UDEV_FILES})
    message(STATUS "Appending contents of udev/${FILENAME} to debian/${DEBIAN_PACKAGE_NAME}.udev")
    file(READ udev/${FILENAME} FILEDATA)
    file(APPEND ${CMAKE_CURRENT_SOURCE_DIR}/debian/${DEBIAN_PACKAGE_NAME}.udev ${FILEDATA})
  endforeach(FILENAME)
else()
  install(DIRECTORY udev
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endif()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/bumblebee.launch)
  roslaunch_add_file_check(launch/camera.launch)
  roslaunch_add_file_check(launch/stereo.launch)

  find_package(roslint REQUIRED)
  roslint_cpp()
endif()
