#!/bin/sh
yes | rosclean purge
sleep 5;

gnome-terminal --tab -e "bash -c 'cd ~/obstacle_detection_ws;python3 launch_modules.py'"