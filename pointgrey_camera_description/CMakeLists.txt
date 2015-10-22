cmake_minimum_required(VERSION 2.8.3)
project(pointgrey_camera_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY meshes urdf launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
