cmake_minimum_required(VERSION 2.8.3)
project(image_exposure_msgs)

find_package(catkin REQUIRED COMPONENTS statistics_msgs std_msgs message_generation)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  ExposureSequence.msg
  ImageExposureStatistics.msg
  SequenceExposureStatistics.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  statistics_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS statistics_msgs std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
