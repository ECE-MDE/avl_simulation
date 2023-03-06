#!/bin/bash
#===============================================================================
# Autonomous Vehicle Library
#
# Description: This bash script kills running processes named roslaunch until
#              there are no more roslaunch processes.
#===============================================================================

killall -9 roscore
killall -9 rosmaster

# Loop and kill processes called roslaunch until there are none left
while true ; do

    ROS_RUNNING=$(ps -e | pgrep roslaunch)

    if [ -z ${ROS_RUNNING} ]
    then
          echo "avl_stop.sh: no roslaunch process to kill"
          break
    else
          echo "avl_stop.sh: found roslaunch process to kill"
          kill $(ps -e | pgrep roslaunch)
    fi

    # Sleep for 1 second
    sleep 1

done
