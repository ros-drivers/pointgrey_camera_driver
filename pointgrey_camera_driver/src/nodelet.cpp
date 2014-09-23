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
   @file nodelet.cpp
   @author Chad Rockey
   @date July 13, 2011
   @brief ROS nodelet for the Point Grey Chameleon Camera

   @attention Copyright (C) 2011
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
*/

// ROS and associated nodelet interface and PLUGINLIB declaration header
#include "ros/ros.h"
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "pointgrey_camera_driver/PointGreyCamera.h" // The actual standalone library for the PointGreys

#include <image_transport/image_transport.h> // ROS library that allows sending compressed images
#include <camera_info_manager/camera_info_manager.h> // ROS library that publishes CameraInfo topics
#include <sensor_msgs/CameraInfo.h> // ROS message header for CameraInfo

#include <wfov_camera_msgs/WFOVImage.h>
#include <image_exposure_msgs/ExposureSequence.h> // Message type for configuring gain and white balance.

#include <diagnostic_updater/diagnostic_updater.h> // Headers for publishing diagnostic messages.
#include <diagnostic_updater/publisher.h>

#include <boost/thread.hpp> // Needed for the nodelet to launch the reading thread.

#include <dynamic_reconfigure/server.h> // Needed for the dynamic_reconfigure gui service to run

namespace pointgrey_camera_driver
{

class PointGreyCameraNodelet: public nodelet::Nodelet
{
public:
  PointGreyCameraNodelet() {}

  ~PointGreyCameraNodelet()
  {
    if(pubThread_)
    {
      pubThread_->interrupt();
      pubThread_->join();
    }

    try
    {
      NODELET_DEBUG("Stopping camera capture.");
      pg_.stop();
      NODELET_DEBUG("Disconnecting from camera.");
      pg_.disconnect();
    }
    catch(std::runtime_error& e)
    {
      NODELET_ERROR("%s", e.what());
    }
  }

private:
  /*!
  * \brief Function that allows reconfiguration of the camera.
  *
  * This function serves as a callback for the dynamic reconfigure service.  It simply passes the configuration object to the driver to allow the camera to reconfigure.
  * \param config  camera_library::CameraConfig object passed by reference.  Values will be changed to those the driver is currently using.
  * \param level driver_base reconfiguration level.  See driver_base/SensorLevels.h for more information.
  */
  void paramCallback(pointgrey_camera_driver::PointGreyConfig &config, uint32_t level)
  {
    try
    {
      NODELET_DEBUG("Dynamic reconfigure callback with level: %d", level);
      pg_.setNewConfiguration(config, level);

      // Store needed parameters for the metadata message
      gain_ = config.gain;
      wb_blue_ = config.white_balance_blue;
      wb_red_ = config.white_balance_red;

      // Store CameraInfo binning information
      binning_x_ = 1;
      binning_y_ = 1;
      /*     
      if(config.video_mode == "640x480_mono8" || config.video_mode == "format7_mode1")
      {
        binning_x_ = 2;
        binning_y_ = 2;
      }
      else if(config.video_mode == "format7_mode2")
      {
        binning_x_ = 0;
        binning_y_ = 2;
      }
      else
      {
        binning_x_ = 0;
        binning_y_ = 0;
      }
      */

      // Store CameraInfo RegionOfInterest information
      if(config.video_mode == "format7_mode0" || config.video_mode == "format7_mode1" || config.video_mode == "format7_mode2")
      {
        roi_x_offset_ = config.format7_x_offset;
        roi_y_offset_ = config.format7_y_offset;
        roi_width_ = config.format7_roi_width;
        roi_height_ = config.format7_roi_height;
        do_rectify_ = true; // Set to true if an ROI is used.
      }
      else
      {
        // Zeros mean the full resolution was captured.
        roi_x_offset_ = 0;
        roi_y_offset_ = 0;
        roi_height_ = 0;
        roi_width_ = 0;
        do_rectify_ = false; // Set to false if the whole image is captured.
      }
    }
    catch(std::runtime_error& e)
    {
      NODELET_ERROR("Reconfigure Callback failed with error: %s", e.what());
    }
  }

  /*!
  * \brief Connection callback to only do work when someone is listening.
  *
  * This function will connect/disconnect from the camera depending on who is using the output.
  */
  void connectCb()
  {
    NODELET_DEBUG("Connect callback!");
    boost::mutex::scoped_lock scopedLock(connect_mutex_); // Grab the mutex.  Wait until we're done initializing before letting this function through.
    // Check if we should disconnect (there are 0 subscribers to our data)
    if(it_pub_.getNumSubscribers() == 0 && pub_->getPublisher().getNumSubscribers() == 0)
    {
      try
      {
        NODELET_DEBUG("Disconnecting.");
        pubThread_->interrupt();
        pubThread_->join();
        sub_.shutdown();
        NODELET_DEBUG("Stopping camera capture.");
        pg_.stop();
        /*NODELET_DEBUG("Disconnecting from camera.");
        pg_.disconnect();*/
      }
      catch(std::runtime_error& e)
      {
        NODELET_ERROR("%s", e.what());
      }
    }
    else if(!sub_)     // We need to connect
    {
      NODELET_DEBUG("Connecting");
      // Try connecting to the camera
      volatile bool connected = false;
      while(!connected && ros::ok())
      {
        try
        {
          NODELET_DEBUG("Connecting to camera.");
          pg_.connect(); // Probably already connected from the reconfigure thread.  This will will not throw if successfully connected.
          connected = true;
        }
        catch(std::runtime_error& e)
        {
          NODELET_ERROR("%s", e.what());
          ros::Duration(1.0).sleep(); // sleep for one second each time
        }
      }

      // Set the timeout for grabbing images.
      double timeout;
      getMTPrivateNodeHandle().param("timeout", timeout, 1.0);
      try
      {
        NODELET_DEBUG("Setting timeout to: %f.", timeout);
        pg_.setTimeout(timeout);
      }
      catch(std::runtime_error& e)
      {
        NODELET_ERROR("%s", e.what());
      }

      // Subscribe to gain and white balance changes
      sub_ = getMTNodeHandle().subscribe("image_exposure_sequence", 10, &pointgrey_camera_driver::PointGreyCameraNodelet::gainWBCallback, this);

      volatile bool started = false;
      while(!started && ros::ok())
      {
        try
        {
          NODELET_DEBUG("Starting camera capture.");
          pg_.start();
          started = true;
        }
        catch(std::runtime_error& e)
        {
          NODELET_ERROR("%s", e.what());
          ros::Duration(1.0).sleep(); // sleep for one second each time
        }
      }

      // Start the thread to loop through and publish messages
      pubThread_.reset(new boost::thread(boost::bind(&pointgrey_camera_driver::PointGreyCameraNodelet::devicePoll, this)));
    }
    else
    {
      NODELET_DEBUG("Do nothing in callback.");
    }
  }

  /*!
  * \brief Serves as a psuedo constructor for nodelets.
  *
  * This function needs to do the MINIMUM amount of work to get the nodelet running.  Nodelets should not call blocking functions here.
  */
  void onInit()
  {
    // Get nodeHandles
    ros::NodeHandle &nh = getMTNodeHandle();
    ros::NodeHandle &pnh = getMTPrivateNodeHandle();

    // Get a serial number through ros
    int serial;
    pnh.param<int>("serial", serial, 0);
    pg_.setDesiredCamera((uint32_t)serial);

    // Get the location of our camera config yaml
    std::string camera_info_url;
    pnh.param<std::string>("camera_info_url", camera_info_url, "");
    // Get the desired frame_id, set to 'camera' if not found
    pnh.param<std::string>("frame_id", frame_id_, "camera");

    // Do not call the connectCb function until after we are done initializing.
    boost::mutex::scoped_lock scopedLock(connect_mutex_);

    // Start up the dynamic_reconfigure service, note that this needs to stick around after this function ends
    srv_ = boost::make_shared <dynamic_reconfigure::Server<pointgrey_camera_driver::PointGreyConfig> > (pnh);
    dynamic_reconfigure::Server<pointgrey_camera_driver::PointGreyConfig>::CallbackType f =  boost::bind(&pointgrey_camera_driver::PointGreyCameraNodelet::paramCallback, this, _1, _2);
    srv_->setCallback(f);

    // Start the camera info manager and attempt to load any configurations
    std::stringstream cinfo_name;
    cinfo_name << serial;
    cinfo_.reset(new camera_info_manager::CameraInfoManager(nh, cinfo_name.str(), camera_info_url));

    // Publish topics using ImageTransport through camera_info_manager (gives cool things like compression)
    it_.reset(new image_transport::ImageTransport(nh));
    image_transport::SubscriberStatusCallback cb = boost::bind(&PointGreyCameraNodelet::connectCb, this);
    it_pub_ = it_->advertiseCamera("image_raw", 5, cb, cb);

    // Set up diagnostics
    updater_.setHardwareID("pointgrey_camera " + cinfo_name.str());

    // Set up a diagnosed publisher
    double desired_freq;
    pnh.param<double>("desired_freq", desired_freq, 7.0);
    pnh.param<double>("min_freq", min_freq_, desired_freq);
    pnh.param<double>("max_freq", max_freq_, desired_freq);
    double freq_tolerance; // Tolerance before stating error on publish frequency, fractional percent of desired frequencies.
    pnh.param<double>("freq_tolerance", freq_tolerance, 0.1);
    int window_size; // Number of samples to consider in frequency
    pnh.param<int>("window_size", window_size, 100);
    double min_acceptable; // The minimum publishing delay (in seconds) before warning.  Negative values mean future dated messages.
    pnh.param<double>("min_acceptable_delay", min_acceptable, 0.0);
    double max_acceptable; // The maximum publishing delay (in seconds) before warning.
    pnh.param<double>("max_acceptable_delay", max_acceptable, 0.2);
    ros::SubscriberStatusCallback cb2 = boost::bind(&PointGreyCameraNodelet::connectCb, this);
    pub_.reset(new diagnostic_updater::DiagnosedPublisher<wfov_camera_msgs::WFOVImage>(nh.advertise<wfov_camera_msgs::WFOVImage>("image", 5, cb2, cb2),
               updater_,
               diagnostic_updater::FrequencyStatusParam(&min_freq_, &max_freq_, freq_tolerance, window_size),
               diagnostic_updater::TimeStampStatusParam(min_acceptable, max_acceptable)));
  }

  /*!
  * \brief Function for the boost::thread to grabImages and publish them.
  *
  * This function continues until the thread is interupted.  Responsible for getting sensor_msgs::Image and publishing them.
  */
  void devicePoll()
  {
    while(!boost::this_thread::interruption_requested())   // Block until we need to stop this thread.
    {
      try
      {
        wfov_camera_msgs::WFOVImagePtr wfov_image(new wfov_camera_msgs::WFOVImage);
        // Get the image from the camera library
        NODELET_DEBUG("Starting a new grab from camera.");
        pg_.grabImage(wfov_image->image, frame_id_);

        // Set other values
        wfov_image->header.frame_id = frame_id_;

        wfov_image->gain = gain_;
        wfov_image->white_balance_blue = wb_blue_;
        wfov_image->white_balance_red = wb_red_;

        wfov_image->temperature = pg_.getCameraTemperature();

        ros::Time time = ros::Time::now();
        wfov_image->header.stamp = time;
        wfov_image->image.header.stamp = time;

        // Set the CameraInfo message
        ci_.reset(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
        ci_->header.stamp = wfov_image->image.header.stamp;
        ci_->header.frame_id = wfov_image->header.frame_id;
        // The height, width, distortion model, and parameters are all filled in by camera info manager.
        ci_->binning_x = binning_x_;
        ci_->binning_y = binning_y_;
        ci_->roi.x_offset = roi_x_offset_;
        ci_->roi.y_offset = roi_y_offset_;
        ci_->roi.height = roi_height_;
        ci_->roi.width = roi_width_;
        ci_->roi.do_rectify = do_rectify_;

        wfov_image->info = *ci_;

        // Publish the full message
        if(pub_->getPublisher().getNumSubscribers() > 0)
        {
          pub_->publish(wfov_image);
        }

        // Publish the message using standard image transport
        if(it_pub_.getNumSubscribers() > 0)
        {
          sensor_msgs::ImagePtr image(new sensor_msgs::Image(wfov_image->image));
          it_pub_.publish(image, ci_);
        }


      }
      catch(CameraTimeoutException& e)
      {
        NODELET_WARN("%s", e.what());
      }
      catch(std::runtime_error& e)
      {
        NODELET_ERROR("%s", e.what());
        ///< @todo Look into readding this
        /*try{
          // Something terrible has happened, so let's just disconnect and reconnect to see if we can recover.
          pg_.disconnect();
          ros::Duration(1.0).sleep(); // sleep for one second each time
          pg_.connect();
          pg_.start();
        } catch(std::runtime_error& e2){
          NODELET_ERROR("%s", e2.what());
        }*/
      }
      // Update diagnostics
      updater_.update();
    }
    NODELET_DEBUG("Leaving thread.");
  }

  void gainWBCallback(const image_exposure_msgs::ExposureSequence &msg)
  {
    try
    {
      NODELET_DEBUG("Gain callback:  Setting gain to %f and white balances to %u, %u", msg.gain, msg.white_balance_blue, msg.white_balance_red);
      gain_ = msg.gain;
      pg_.setGain(gain_);
      wb_blue_ = msg.white_balance_blue;
      wb_red_ = msg.white_balance_red;
      pg_.setBRWhiteBalance(wb_blue_, wb_red_);
    }
    catch(std::runtime_error& e)
    {
      NODELET_ERROR("gainWBCallback failed with error: %s", e.what());
    }
  }

  boost::shared_ptr<dynamic_reconfigure::Server<pointgrey_camera_driver::PointGreyConfig> > srv_; ///< Needed to initialize and keep the dynamic_reconfigure::Server in scope.
  boost::shared_ptr<image_transport::ImageTransport> it_; ///< Needed to initialize and keep the ImageTransport in scope.
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_; ///< Needed to initialize and keep the CameraInfoManager in scope.
  image_transport::CameraPublisher it_pub_; ///< CameraInfoManager ROS publisher
  boost::shared_ptr<diagnostic_updater::DiagnosedPublisher<wfov_camera_msgs::WFOVImage> > pub_; ///< Diagnosed publisher, has to be a pointer because of constructor requirements
  ros::Subscriber sub_; ///< Subscriber for gain and white balance changes.

  boost::mutex connect_mutex_;

  diagnostic_updater::Updater updater_; ///< Handles publishing diagnostics messages.
  double min_freq_;
  double max_freq_;

  PointGreyCamera pg_; ///< Instance of the PointGreyCamera library, used to interface with the hardware.
  sensor_msgs::CameraInfoPtr ci_; ///< Camera Info message.
  std::string frame_id_; ///< Frame id for the camera messages, defaults to 'camera'
  boost::shared_ptr<boost::thread> pubThread_; ///< The thread that reads and publishes the images.

  double gain_;
  uint16_t wb_blue_;
  uint16_t wb_red_;

  // Parameters for cameraInfo
  size_t binning_x_; ///< Camera Info pixel binning along the image x axis.
  size_t binning_y_; ///< Camera Info pixel binning along the image y axis.
  size_t roi_x_offset_; ///< Camera Info ROI x offset
  size_t roi_y_offset_; ///< Camera Info ROI y offset
  size_t roi_height_; ///< Camera Info ROI height
  size_t roi_width_; ///< Camera Info ROI width
  bool do_rectify_; ///< Whether or not to rectify as if part of an image.  Set to false if whole image, and true if in ROI mode.
};

PLUGINLIB_DECLARE_CLASS(pointgrey_camera_driver, PointGreyCameraNodelet, pointgrey_camera_driver::PointGreyCameraNodelet, nodelet::Nodelet);  // Needed for Nodelet declaration
}
