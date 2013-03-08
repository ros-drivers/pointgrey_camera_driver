/**This code was developed by the National Robotics Engineering Center (NREC),
part of Carnegie Mellon University's Robotics Institute.  Its development was
funded by DARPA under the LS3 program and submitted for public release
on June 7th, 2012 (granted on <NEWDATE_RELEASE_REPLACE>).*/


/**
   @file stereo_node.cpp
   @author Chad Rockey (chadrockey@nrec.ri.cmu.edu)
   @date March 20, 2012
   @brief ROS node to wrap the nodelet for standalone rosrun execution
   
   @attention Copyright (C) 2012
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
   @attention All rights reserved
   @attention Carnegie Mellon Proprietary
   @attention Government Purpose Rights
*/

#include "ros/ros.h"
#include <nodelet/loader.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "pointgrey_camera_node");
  
  // This is code based nodelet loading, the preferred nodelet launching is done through roslaunch
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  nodelet.load("pointgrey_camera_node", "pointgrey_camera_driver/PointGreyStereoCameraNodelet", remap, nargv);
  
  ros::spin();

  return 0;
}