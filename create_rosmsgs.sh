#!/bin/bash -e
# Copyright 2017 University of Cincinnati. All rights reserved. See LICENSE.md file at:
# https://github.com/cmcghan/soi_tools
# Additional copyright may be held by others, as reflected in the commit history.

echo "creating ROS .msg files from MDMs in '/home/$USER/UxAS_$USER/OpenUxAS/mdms'..." 
echo "and storing them in directory ./catkin_lmcp (at the root of whatever directory this script is run from!)"
./lmcp2rosmsg/lmcp2rosmsg.py dir /home/$USER/UxAS_$USER/OpenUxAS/mdms
echo " "
echo "All done!"
echo "Remember to run 'catkin_make' in the 'catkin_lmcp' directory for ROS to see what's going on!"
