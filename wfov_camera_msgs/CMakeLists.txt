cmake_minimum_required(VERSION 2.8.3)
project(wfov_camera_msgs)

find_package(catkin REQUIRED COMPONENTS sensor_msgs message_generation)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  WFOVCompressedImage.msg
  WFOVImage.msg
  WFOVTrigger.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS sensor_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
