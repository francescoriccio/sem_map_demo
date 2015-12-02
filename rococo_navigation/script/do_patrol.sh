#!/bin/bash


# Patrol
rosrun rococo_navigation turn_node -client $1 0 ABS
rosrun rococo_navigation follow_corridor_node -client $1 15 2
rosrun rococo_navigation turn_node -client $1 180 ABS
rosrun rococo_navigation follow_corridor_node -client $1 2 2
rosrun rococo_navigation turn_node -client $1 90 ABS
rosrun rococo_navigation follow_corridor_node -client $1 2 15
rosrun rococo_navigation turn_node -client $1 -90 ABS
rosrun rococo_navigation follow_corridor_node -client $1 2 2


rosrun rococo_navigation turn_node -client $1 0 ABS
rosrun rococo_navigation follow_corridor_node -client $1 15 2
rosrun rococo_navigation turn_node -client $1 180 ABS
rosrun rococo_navigation follow_corridor_node -client $1 2 2
rosrun rococo_navigation turn_node -client $1 90 ABS
rosrun rococo_navigation follow_corridor_node -client $1 2 15
rosrun rococo_navigation turn_node -client $1 -90 ABS
rosrun rococo_navigation follow_corridor_node -client $1 2 2
