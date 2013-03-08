/**This code was developed by the National Robotics Engineering Center (NREC),
part of Carnegie Mellon University's Robotics Institute.  Its development was
funded by DARPA under the LS3 program and submitted for public release
on June 7th, 2012 (granted on <NEWDATE_RELEASE_REPLACE>).*/


/*-*-C++-*-*/
/**
   @file camera_exceptions.h
   @author Chad Rockey (chadrockey@nrec.ri.cmu.edu)
   @date July 20, 2011
   @brief Interface to Point Grey cameras
   
   @attention Copyright (C) 2011
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
   @attention All rights reserved
   @attention Carnegie Mellon Proprietary
   @attention Government Purpose Rights
*/

#ifndef _CAMERAEXCEPTIONS_H_
#define _CAMERAEXCEPTIONS_H_

#include <stdexcept>

class CameraTimeoutException: public std::runtime_error{
  public:
    CameraTimeoutException():runtime_error("Image not found within timeout."){}
    CameraTimeoutException(std::string msg):runtime_error(msg.c_str()){}
};

class CameraNotRunningException: public std::runtime_error{
  public:
    CameraNotRunningException():runtime_error("Camera is currently not running.  Please start the capture."){}
    CameraNotRunningException(std::string msg):runtime_error(msg.c_str()){}
};

class CameraImageNotReadyException: public std::runtime_error{
  public:
    CameraImageNotReadyException():runtime_error("Image is currently not ready."){}
    CameraImageNotReadyException(std::string msg):runtime_error(msg.c_str()){}
};

#endif