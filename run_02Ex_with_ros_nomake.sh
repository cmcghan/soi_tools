#!/bin/bash
# Copyright 2017 University of Cincinnati. All rights reserved. See LICENSE.md file at:
# https://github.com/cmcghan/soi_tools
# Additional copyright may be held by others, as reflected in the commit history.

# do NOT put () in bash -c call, will not work!

# references for looping over arrays:
# * https://stackoverflow.com/questions/8880603/loop-through-an-array-of-strings-in-bash
# * http://tldp.org/LDP/abs/html/loops1.html
# reference for bash strings to gnome-terminal -e:
# * https://stackoverflow.com/questions/32276623/bash-passing-strings-to-gnome-terminal-e


if [ $# -lt 1 ] || [ [ "$1" != "ros" ] && [ "$1" != "nomake" ] && [ "$1" != "tbot" ] && [ "$1" != "nomaketbot" ] ]; then
    echo "This script requires at least one valid argument!"
    echo "Acceptable arguments are: 'ros' 'nomake' 'tbot' 'nomaketbot'"
fi

if [ "$1" == "ros" ] || [ "$1" == "tbot" ]; then
    # "cd commands; source and other commands" "run command" "label"
    PAUSERUN=("./lmcp2rosmsg/lmcp2rosmsg.py dir /home/$USER/UxAS_$USER/OpenUxAS/mdms; cd catkin_lmcp; catkin_make; source devel/setup.bash" "roscore" "roscore")
elif [ "$1" == "nomake" ] || [ "$1" == "nomaketbot" ]; then
    # "cd commands; source and other commands" "run command" "label"
    PAUSERUN=("cd catkin_lmcp; source devel/setup.bash" "roscore" "roscore")
fi
PAUSERUN+=("cd catkin_lmcp; source devel/setup.bash" "roslaunch ../../rss_git_lite/rosbridge_server_9090.launch" "roslaunch")

# "cd commands; source and other commands" "run command"
LISTRUN=(
"cd catkin_lmcp; source devel/setup.bash" "rostopic echo /from_uxas/MissionCommand"
"cd catkin_lmcp; source devel/setup.bash" "rostopic echo /from_uxas/LineSearchTask"
"cd catkin_lmcp; source devel/setup.bash" "rostopic echo /from_uxas/VehicleActionCommand"
)
if [ "$1" == "ros" ] || [ "$1" == "nomake" ]; then
    LISTRUN+=(
    "cd ../OpenUxAS/examples/02_Example_WaterwaySearch" "./runUxAS_WaterwaySearch.sh"
    "cd ../OpenUxAS/examples/02_Example_WaterwaySearch" "./runAMASE_WaterwaySearch.sh"
    "cd ../OpenUxAS/examples/02_Example_WaterwaySearch/ext_logger" "python ./ros_adapter.py"
    )
elif [ "$1" == "tbot" ] || [ "$1" == "nomaketbot" ]; then
    # turtlebot calls modified from: http://learn.turtlebot.com/2015/02/03/7/

    # "cd commands; source and other commands" "run command"
    LISTRUN+=(
    "cd ../OpenUxAS/examples/02_Example_WaterwaySearch" "./runUxAS_WaterwaySearch.sh"
    "cd catkin_lmcp; source devel/setup.bash" "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/opt/ros/kinetic/share/turtlebot_gazebo/worlds/empty.world"
#    "cd catkin_lmcp; source devel/setup.bash" "roslaunch turtlebot_gazebo amcl_demo.launch"
#    "cd catkin_lmcp; source devel/setup.bash" "rosrun tf tf_echo /map /base_link"
    "cd ../OpenUxAS/examples/02_Example_WaterwaySearch/ext_logger" "python ./ros_adapter.py"
    )
fi
#echo LISTRUN is $LISTRUN
#echo LISTRUN(all) is ${LISTRUN[@]}

# length via ${#name[@]}
for (( i=0; i<${#PAUSERUN[@]}; i+=3 ))
do
    eval "gnome-terminal -e 'bash -c \"echo ${PAUSERUN[$i+1]}; ${PAUSERUN[$i]}; ${PAUSERUN[$i+1]}; read line\"' "

    while [ 1 ]
    do
        if [ `ps -A | grep -c ${PAUSERUN[$i+2]}` -eq 1 ]; then
            break
        else
            echo "Waiting until ${PAUSERUN[$i+2]} is up before proceeding..."
            sleep 2
        fi
    done
done

# length via ${#name[@]}
for (( i=0; i<${#LISTRUN[@]}; i+=2 ))
do
    #RUNTHIS="-e 'bash -c \"echo ${LISTRUN[$i+1]}; ${LISTRUN[$i]}; ${LISTRUN[$i+1]}; read line\"' "
    #eval "gnome-terminal "$RUNTHIS
    eval "gnome-terminal -e 'bash -c \"echo ${LISTRUN[$i+1]}; ${LISTRUN[$i]}; ${LISTRUN[$i+1]}; read line\"' "
done

# --eof--

