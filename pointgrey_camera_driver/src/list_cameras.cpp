/*
This code was developed by the National Robotics Engineering Center (NREC), part of the Robotics Institute at Carnegie Mellon University.
Its development was funded by DARPA under the LS3 program and submitted for public release on June 7th, 2012.
Release was granted on August, 21st 2012 with Distribution Statement "A" (Approved for Public Release, Distribution Unlimited).

This software is released under a BSD license:

Copyright (c) 2012, Carnegie Mellon University. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
Neither the name of the Carnegie Mellon University nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


/**
   @file list_cameras.cpp
   @author Chad Rockey
   @date January 10, 2012
   @brief Executable that lists the attached pointgrey cameras and exits.

   @attention Copyright (C) 2012
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
*/

#include "ros/ros.h"
#include "pointgrey_camera_driver/PointGreyCamera.h"
#include <iostream>

int main(int argc, char **argv)
{
  PointGreyCamera pg_;

  try
  {
    std::vector<uint32_t> serials_list = pg_.getAttachedCameras();
    if(serials_list.size() == 0)
    {
      std::cout << "No PointGrey cameras detected on this computer." << std::endl;

      std::cout << std::endl
                << "Note that you may need to restart udev and replug your camera, eg:" << std::endl
                << "  sudo service udev restart" << std::endl;
    }
    else
    {
      std::cout << "The following camera serials are connected on this computer:" << std::endl;
      for(uint i = 0; i < serials_list.size(); i++)
      {
        std::cout << serials_list[i] << std::endl;
      }
    }
  }
  catch(std::runtime_error& e)
  {
    std::cout << "There was an error checking the active cameras: " << e.what() << std::endl;
  }

  return 0;
}
