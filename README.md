# The rotoscan metapackage
This contains the driver and necessary utilities for Leuze rotoScan laser rangefinder device.

## rotoscan_node

A ROS Driver for Leuze rotoScan laser rangefinder devices, publishing sensor_msgs/LaserScan messages. Currently supports devices connected via Ethernet and serial port, but could be extended to interface with other models.
Orginially from [here](https://svn-agbkb.informatik.uni-bremen.de/dfki-sks-ros-pkg/trunk/dfki_sks_laser_drivers/rotoscan_node) and updated for the catkin build system.

## timeutils

timeutils contains utility code related to time(stamps) on *NIX.
Orginially from [here](https://svn-agbkb.informatik.uni-bremen.de/dfki-sks-ros-pkg/trunk/dfki_sks_laser_drivers/) and updated for the catkin build system.
