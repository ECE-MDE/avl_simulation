#!/bin/bash
#==============================================================================
# Autonomous Vehicle Library
#
# Description: This bash script handles the startup of an AVL roslaunch
#              instance, including the creation of a log folder and the calling
#              of roslaunch. The following are the tasks handles by this script,
#              in order:
#                  1) Create a new log folder whose name is formatted as
#                     YYYY-MM-DD_HH.MM.SS
#                     in /var/avl_logs/.
#                  2) Make a symlink from the folder /var/avl_logs/current to
#                     the log folder that was just created.
#                  3) Copy all .config files from the avl repository directory
#                     into the config folder in the log folder that was just
#                     created.
#                  4) Executes the ROS launch file specified by the arguments to
#                     this script using roslaunch.
#                  5) Exits.
#              Note: The AVL catkin workspace must be sourced before running
#              this script. If it is not, roslaunch will fail to find the
#              packages and launch files.
#
# Usage:       ./avl_sim_start [package_name] [launch_file_name]
#
# Options:     -package_name: The name of the ROS package containing the desired
#                          launch file. (required)
#              -launch_file_name: The name of launch file to launch. (required)
#===============================================================================

# Ensure that there are two arguments
if [[ "$#" != 1 ]] ; then
  echo "avl_sim_start.sh: usage: ./avl_sim_start [# simulated vehicles]"
  exit 1
fi

if (ps -e | grep roslaunch); then
    echo "Roslaunch process found running. Try avl_restart instead."
    exit 1
fi

# Check to see if /var/avl_config directory exists. If not, make it
if !  [[ -d "/var/avl_config" ]] ; then
  echo "avl_sim_start.sh: creating /var/avl_config directory"
  sudo mkdir "/var/avl_config"
  sudo chown -R "${USER}:${USER}" /var/avl_config
else
  echo "avl_sim_start.sh: /var/avl_config directory exists"
fi

# Create directories for each simulated vehicle
for (( i=1; i<=${1}; i++ ))
do
    # Check to see if /var/avl_config directory exists. If not, make it
    if !  [[ -d "/var/avl_config/$i" ]] ; then
      echo "avl_sim_setup.sh: No /var/avl_config/$i directory. Please run ./avl_sim_setup.sh ${1}"
      exit 1
    else
      echo "avl_sim_setup.sh: /var/avl_config/$i directory exists"
    fi
done

# Check to see if /var/avl_logs directory exists. If not, make it
if !  [[ -d "/var/avl_logs" ]] ; then
  echo "avl_sim_start.sh: creating /var/avl_logs directory"
  sudo mkdir "/var/avl_logs"
  sudo chown -R "${USER}:${USER}" /var/avl_logs
else
  echo "avl_sim_start.sh: /var/avl_logs directory exists"
fi

# Grab the current date and time formatted as
#     YYYY-MM-DD_HH.MM.SS
# and create a new log folder in /var/avl_logs whose name is the data and time.
# In the log folder, create the log and config folders
DATEANDTIME=`date +%F_%H.%M.%S`
mkdir "/var/avl_logs/${DATEANDTIME}"
mkdir "/var/avl_logs/${DATEANDTIME}/log"
# Launch each vehicle
for (( i=1; i<=${1}; i++ ))
do
    mkdir "/var/avl_logs/${DATEANDTIME}/log/$i"
done
mkdir "/var/avl_logs/${DATEANDTIME}/config"
echo "avl_sim_start.sh: created /var/avl_logs/${DATEANDTIME}"
echo "avl_sim_start.sh: created /var/avl_logs/${DATEANDTIME}/log"
echo "avl_sim_start.sh: created /var/avl_logs/${DATEANDTIME}/config"

# Create a symbolic link that points from current to the timestamped log
# folder that we just created.
#    -s creates a symbolic link
#    -f removes the existing destination if it exists
#    -n treats the destination as a normal file instead of a link
ln -sfn "/var/avl_logs/${DATEANDTIME}" "/var/avl_logs/current"
echo "avl_sim_start.sh: created symbolic link (/var/avl_logs/current -> /var/avl_logs/${DATEANDTIME})"

#    Copy all config files into the avl_logs
cp -a "/var/avl_config/." "/var/avl_logs/current/config"

# End subshell, return to user directory
echo "avl_sim_start.sh: copied .config files to /var/avl_logs/${DATEANDTIME}/config"

# Start the roscore instance and give some time for things to start correctly
roscore &
sleep 3

# Launch the simulation time for all vehicles
roslaunch "avl_simulation" "sim_time.launch" &

# Launch each vehicle
for (( i=1; i<=${1}; i++ ))
do
    roslaunch "avl_simulation" "multi_simulation.launch" "my_ns:=$i" "vehicle_id:=$i" &
done

# Successful exit
echo "avl_sim_start.sh: roslaunch instance started"
exit 0
