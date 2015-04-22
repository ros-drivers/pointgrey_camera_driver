^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pointgrey_camera_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.12.0 (2015-04-22)
-------------------
* Improve list_cameras by outputing more information about it
  The previous list_cameras only output 1 serial number which is
  not very informative. The improved one will print serial, model,
  vendor, sensor, resolution, color and firmware version.
* Add auto white balance and fix not able to write white balance
  Fix the problem of not being able to set white balance using Property.
  When trying to set white balance on my FL3-U3-13E4C-C, both this ros
  driver and flycap software cannot set the white balance naturally.
  After playing around with the flycap software, I found that I have
  to set the highest bit of register 80C (which is white balance) to 1
  to enable this feature. So I modified the original SetWhiteBalance to
  use WriteRegister directly. And add support for auto white balance.
* Support ARM flycapture SDK.
* Downgrade flycapture SDK to 2.6.3.4, see:
  https://github.com/ros-drivers/pointgrey_camera_driver/issues/28
* Contributors: Chao Qu, Julius Gelšvartas, Konrad Banachowicz, L0g1x, Mike Purvis

0.11.0 (2014-11-07)
-------------------
* Change approach to downloading flycapture SDK.
  The logic which fetches and extracts the archive from ptgrey.com
  has been moved to a more reasonable and comprehensible python script.
  This should better pave the way for better future ARM support in this
  driver.
* Use dh_installudev for udev rules.
* The raw and mono pixel formats (raw8, raw16, mono8, mono16) can be selected from dynamic reconfigure with every video mode (while before it was hard coded that only mono pixel formats could be used with mode1 and mode2).
* Binning information removed from camera_info published by the nodelet.
* Add image_proc as dependency.
* Removed changes to binning_x and binning_y in camera info messages (otherwise image_proc node would performs a further downsampling).
* Now the wrapper allows to set raw and mono pixel formats with any mode.
* Added possibility to set GigE packet delay as launch/conf parameter.
* Changed 'auto_packet_size' to 'true' as default.
* Added possibility to change GigE packet size for GigE cameras.
* For GigE cameras, automatically discover best packet size.
* Fix launch file syntax error (XML comments neither nest nor continue).
* Contributors: Aaron Denney, Jeff Schmidt, Matteo Munaro, Mike Purvis

0.10.0 (2014-08-18)
-------------------
* Added frame rate parameter to launchfiles.
* Fixing lack of dynamic Bayer format detection/incorrect fixed Bayer format detection in the stereo driver, tested on BB2 hardware
* Should prevent multiple camera nodes from conflicting.
* Read camera's resulting trigger configuration.
* Read camera's resulting strobe configuration.
* Refactor GPIO pin comparison into separate function.
* Support outputting strobes.
* Enable altering trigger polarities.
* Don't overwrite currently unused fields.
* Modify firewire rule per issue `#6 <https://github.com/ros-drivers/pointgrey_camera_driver/issues/6>`_
* Make sure camera properties are supported before enabling them
* Contributors: Aaron Denney, Jake Bruce, Jeff Schmidt, Mike Purvis, Ryan Gariepy

0.9.2 (2014-07-13)
------------------
* Added dpkg to build_depend
  During builds, dpkg is explicitly called. This tool is not necessarily on all systems, so we should make sure it is installed during the build.
* Contributors: Scott K Logan

0.9.1 (2014-03-12)
------------------
* Add note to the list_cameras tool about restarting udev.
* Add debayering nodelet to example launcher for monocular camera. Tested with a USB Firefly.
* Automatic lint fixes from astyle.
* Set ROS message image encoding to the bayer format declared by the camera.
* Contributors: Mike Purvis

0.9.0 (2014-02-26)
------------------
* Remove pgrimaging from all USB devices.
* Rename standalone executables, fix priority of udev rules for USB cameras, parameterize example launchfiles better.
* Contributors: Mike Purvis

0.0.2 (2014-02-26)
------------------
* Permissions to world-readable for firewire devices.
* Add nodelet manager to example launch.
* Reorganize bumblebee example launcher.
* Fix installing to i386.
* Contributors: Mike Purvis

0.0.1 (2014-02-23)
------------------
* Fetch FlyCap dependency from pygrey.com at configure time.
* Add PGR udev rules from the flycap installer.
* Catkinize main package.
* Added code for a ROS-compatible point-grey camera driver based on flycap.
* Contributors: Chad Rockey, Dave Bradley, Mike Purvis
