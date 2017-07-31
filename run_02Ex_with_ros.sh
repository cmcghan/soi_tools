#!/bin/bash
# Copyright 2017 University of Cincinnati. All rights reserved. See LICENSE.md file at:
# https://github.com/cmcghan/soi_tools
# Additional copyright may be held by others, as reflected in the commit history.

gnome-terminal -e 'bash -c "./lmcp2rosmsg/lmcp2rosmsg.py dir /home/$USER/UxAS_$USER/OpenUxAS/mdms; cd catkin_lmcp; catkin_make; source devel/setup.bash; roscore; read line"'

while [ 1 ]
do
    if [ `ps -A | grep -c roscore` -eq 1 ]; then
        break
    else
        echo "Waiting until roscore is up before proceeding..."
        sleep 2
    fi
done

gnome-terminal -e 'bash -c "cd catkin_lmcp; source devel/setup.bash; roslaunch ../../rss_git_lite/rosbridge_server_9090.launch; read line"'

while [ 1 ]
do
    if [ `ps -A | grep -c roslaunch` -eq 1 ]; then
        break
    else
        echo "Waiting until roslaunch of rosbridge_suite is up before proceeding..."
        sleep 2
    fi
done

gnome-terminal -e 'bash -c "cd catkin_lmcp; source devel/setup.bash; rostopic echo /from_uxas/MissionCommand; read line"'

gnome-terminal -e 'bash -c "cd catkin_lmcp; source devel/setup.bash; rostopic echo /from_uxas/LineSearchTask; read line"'

gnome-terminal -e 'bash -c "cd catkin_lmcp; source devel/setup.bash; rostopic echo /from_uxas/VehicleActionCommand; read line"'

gnome-terminal -e 'bash -c "cd ../OpenUxAS/examples/02_Example_WaterwaySearch; ./runUxAS_WaterwaySearch.sh; read line"'

gnome-terminal -e 'bash -c "cd ../OpenUxAS/examples/02_Example_WaterwaySearch; ./runAMASE_WaterwaySearch.sh; read line"'

gnome-terminal -e 'bash -c "cd ../OpenUxAS/examples/02_Example_WaterwaySearch/ext_logger; python ./ros_adapter.py; read line"'

# --eof--
