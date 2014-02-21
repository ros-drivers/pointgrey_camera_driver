cmake_minimum_required(VERSION 2.8.3)
project(pointgrey_camera_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp nodelet sensor_msgs wfov_camera_msgs
  image_exposure_msgs camera_info_manager image_transport
  dynamic_reconfigure driver_base diagnostic_updater)

generate_dynamic_reconfigure_options(
  cfg/PointGrey.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp nodelet
)

include_directories(include ${catkin_INCLUDE_DIRS})

find_library(FLYCAPTURE2 flycapture)
add_library(PointGreyCamera src/PointGreyCamera.cpp)
target_link_libraries(PointGreyCamera ${FLYCAPTURE2} ${catkin_LIBRARIES})
add_dependencies(PointGreyCamera ${PROJECT_NAME}_gencfg) 

add_library(PointGreyCameraNodelet src/nodelet.cpp)
target_link_libraries(PointGreyCameraNodelet PointGreyCamera ${catkin_LIBRARIES})

add_library(PointGreyStereoCameraNodelet src/stereo_nodelet.cpp)
target_link_libraries(PointGreyStereoCameraNodelet PointGreyCamera ${catkin_LIBRARIES})

add_executable(pointgrey_camera_node src/node.cpp)
target_link_libraries(pointgrey_camera_node PointGreyCamera ${catkin_LIBRARIES})

add_executable(pointgrey_stereo_node src/stereo_node.cpp)
target_link_libraries(pointgrey_stereo_node PointGreyCamera ${catkin_LIBRARIES})

add_executable(pointgrey_list_cameras src/list_cameras.cpp)
target_link_libraries(pointgrey_list_cameras PointGreyCamera ${catkin_LIBRARIES})
 
install(TARGETS 
  PointGreyCameraNodelet
  PointGreyStereoCameraNodelet
  pointgrey_camera_node
  pointgrey_stereo_node
  pointgrey_list_cameras 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
