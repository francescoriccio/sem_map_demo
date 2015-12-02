#!/bin/bash
export MAPNAME=`cat ~/.ros/mapname`
echo $MAPNAME
xterm -e roslaunch sapienzbot_reasoning sapienzBot.launch &
sleep 2
xterm -e roslaunch semantic_map_extraction semantic_map_extraction.launch & 
sleep 2

