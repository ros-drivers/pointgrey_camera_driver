/**This code was developed by the National Robotics Engineering Center (NREC),
part of Carnegie Mellon University's Robotics Institute.  Its development was
funded by DARPA under the LS3 program and submitted for public release
on June 7th, 2012 (granted on <NEWDATE_RELEASE_REPLACE>).*/


/**
   @file list_cameras.cpp
   @author Chad Rockey (chadrockey@nrec.ri.cmu.edu)
   @date January 10, 2012
   @brief Executable that lists the attached pointgrey cameras and exits.
   
   @attention Copyright (C) 2012
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
   @attention All rights reserved
   @attention Carnegie Mellon Proprietary
   @attention Government Purpose Rights
*/

#include "ros/ros.h"
#include "pointgrey_camera_driver/PointGreyCamera.h"
#include <iostream>

int main(int argc, char **argv){
  PointGreyCamera pg_;
  
  try{
    std::vector<uint32_t> serials_list = pg_.getAttachedCameras();
    if(serials_list.size() == 0){
      std::cout << "No PointGrey cameras are active on this computer." << std::endl;
    } else {
      std::cout << "The following camera serials are connected on this computer:" << std::endl;
      for(uint i = 0; i < serials_list.size(); i++){
	std::cout << serials_list[i] << std::endl;
      }
    }
  } catch(std::runtime_error& e){
    std::cout << "There was an error checking the active cameras: " << e.what() << std::endl;
  }

  return 0;
}