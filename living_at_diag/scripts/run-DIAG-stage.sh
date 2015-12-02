#!/bin/bash
xterm -e roslaunch living_at_diag launcher.launch &
sleep 3
xterm -e roslaunch sapienzbot_dialog sapienzbot_dialog.launch &
sleep 3
xterm -e roslaunch sapienzbot_reasoning sapienzBot.launch &
sleep 5
xterm -e roslaunch semantic_map_extraction semantic_map_extraction.launch & 



