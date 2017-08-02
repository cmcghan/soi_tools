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

PAUSERUN=(
"cd catkin_lmcp; source devel/setup.bash" "roscore" "roscore"
"cd catkin_lmcp; source devel/setup.bash" "roslaunch ../../rss_git_lite/rosbridge_server_9090.launch" "roslaunch"
)

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

LISTRUN=(
"cd catkin_lmcp; source devel/setup.bash" "rostopic echo /from_uxas/MissionCommand"
"cd catkin_lmcp; source devel/setup.bash" "rostopic echo /from_uxas/LineSearchTask"
"cd catkin_lmcp; source devel/setup.bash" "rostopic echo /from_uxas/VehicleActionCommand"
"cd ../OpenUxAS/examples/02_Example_WaterwaySearch" "./runUxAS_WaterwaySearch.sh"
"cd ../OpenUxAS/examples/02_Example_WaterwaySearch" "./runAMASE_WaterwaySearch.sh"
"cd ../OpenUxAS/examples/02_Example_WaterwaySearch/ext_logger" "python ./ros_adapter.py"
)
#echo LISTRUN is $LISTRUN
#echo LISTRUN(all) is ${LISTRUN[@]}

# length via ${#name[@]}
for (( i=0; i<${#LISTRUN[@]}; i+=2 ))
do
    #RUNTHIS="-e 'bash -c \"echo ${LISTRUN[$i+1]}; ${LISTRUN[$i]}; ${LISTRUN[$i+1]}; read line\"' "
    #eval "gnome-terminal "$RUNTHIS
    eval "gnome-terminal -e 'bash -c \"echo ${LISTRUN[$i+1]}; ${LISTRUN[$i]}; ${LISTRUN[$i+1]}; read line\"' "
done

# --eof--

