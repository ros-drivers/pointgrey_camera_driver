^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pointgrey_camera_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
