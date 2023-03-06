#!/bin/bash
#==============================================================================
# Autonomous Vehicle Library
#
# Description: This script monitors the processes running on the computer and
#              waits for a process named roslaunch to end. When there is no
#              longer a process named rosluanch, roslaunch is started again
#              using the script avl_start.sh. This script must be located in the
#              same directory as avl_stop.sh and avl_start.sh.
#
# Usage:       ./avl_restart [package_name] [launch_file_name]
#
# Options:     -package_name: The name of the ROS package containing the desired
#                          launch file. (required)
#              -launch_file_name: The name of launch file to launch. (required)
#===============================================================================

# Ensure that there are two arguments
if [[ "$#" != 2 ]] ; then
  echo "avl_restart.sh: usage: ./avl_restart [package_name] [launch_file_name]"
  exit 1
fi

# Get the path to this script.
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Execute the avl_stop.sh script to stop roslaunch. Assumes that the avl_stop.sh
# script is in he same directory as this script
echo "avl_restart.sh: runnning avl_stop.sh script"
/bin/bash "${DIR}"/avl_stop.sh

# Execute the avl_start.sh script. Assumes that the avl_start.sh script is in
# the same directory as this script
echo "avl_restart.sh: runnning avl_start.sh script"
/bin/bash "${DIR}"/avl_start.sh "${1}" "${2}"

exit 0
