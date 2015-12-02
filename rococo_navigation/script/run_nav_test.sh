#!/bin/bash

ROBOTNAME="diago_0"

# Navigation test
xterm -e "roslaunch rococo_navigation nav.launch robotname:=$ROBOTNAME" &
sleep 3

# Start rviz
#xterm -e rosrun rviz rviz -d `rospack find stage_environments`/config/rviz/robot_0.rviz &


# Patrol
xterm -e "./do_patrol.sh $ROBOTNAME" &
