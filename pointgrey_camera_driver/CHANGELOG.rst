^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pointgrey_camera_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
