#!/bin/bash
#===============================================================================
# Autonomous Vehicle Library
#
# Description: This bash script copies all .config files from the avl directory
#              to the /var/avl_config directory.
#===============================================================================

# Ensure that there is one argument
if [[ "$#" != 1 ]] ; then
  echo "avl_sim_setup.sh: usage: ./avl_sim_setup [# simulated vehicles]"
  exit 1
fi

# Check to see if /var/avl_config directory exists. If not, make it
if !  [[ -d "/var/avl_config" ]] ; then
  echo "avl_sim_setup.sh: creating /var/avl_config directory"
  sudo mkdir "/var/avl_config"
  sudo chown -R "${USER}:${USER}" /var/avl_config
else
  echo "avl_sim_setup.sh: /var/avl_config directory exists, removing..."
  sudo rm -r /var/avl_config
  echo "avl_sim_setup.sh: creating /var/avl_config directory"
  sudo mkdir "/var/avl_config"
  sudo chown -R "${USER}:${USER}" /var/avl_config
fi

# Check to see if /var/avl_calibration directory exists. If not, make it
if !  [[ -d "/var/avl_calibration" ]] ; then
  echo "avl_sim_setup.sh: creating /var/avl_calibration directory"
  sudo mkdir "/var/avl_calibration"
  sudo chown -R "${USER}:${USER}" /var/avl_calibration
else
  echo "avl_sim_setup.sh: /var/avl_calibration directory exists"
fi

# Create directories for each simulated vehicle
for (( i=1; i<=${1}; i++ ))
do
    # Check to see if /var/avl_config directory exists. If not, make it
    if !  [[ -d "/var/avl_config/$i" ]] ; then
      echo "avl_sim_setup.sh: creating /var/avl_config/$i directory"
      sudo mkdir "/var/avl_config/$i"
      sudo chown -R "${USER}:${USER}" /var/avl_config/$i
    else
      echo "avl_sim_setup.sh: /var/avl_config directory exists"
    fi
done

# Get the path to this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Create a subshell to change the working directory to the avl/src directory
# search for all *.config files, and copy them to  /var/avl_config/#vehicle
(

    # CD into the avl/src directory. This assumes that this script is located
    # in the avl/src/avl_simulation/scripts directory
    cd "${DIR}/../../"
    cp avl_core/include/avl_core/protocol/command_definitions.config /var/avl_config
    cp avl_core/include/avl_core/protocol/action_definitions.config /var/avl_config

    # Create directories for each simulated vehicle
    for (( i=1; i<=${1}; i++ ))
    do
        # The find command finds all files with the .config extension in the current
        # directory and its subdirectories
        #   -type f sets the search type to files
        #   -iname *.config finds all files with the .config extension
        #   -and -not -path ".*/config/*" ignores all files in a folder named
        #       config. This is so that if there are config files in the plot_me
        #       directory in the AVL log plotter, they will not be copied to the
        #       avl_config folder
        #   -print0 prints a null character instead of the default newline
        # The xargs takes the output of the pipe and passes the results to the cp
        # command
        #   -0 indicates items are null terminated
        #   -n 1 specifies one argument per command line
        #   -Ito replaces "to" with the items from the find results
        CONFIG_DIR="/var/avl_config/$i"
        find -type f -iname *.config -and -not -path ".*/config/*" -print0 | xargs -0 -n 1 -Ito cp to $CONFIG_DIR
    done

    if !  [[ -e "/etc/udev/rules.d/99-avl_devices.rules" ]] ; then
      echo "avl_setup.sh: copying 99-avl_devices.rules to /etc/udev/rules.d"
      find -type f -name "99-avl_devices.rules" | xargs -If sudo  cp f /etc/udev/rules.d
    else
      echo "avl_setup.sh: /etc/udev/rules.d/99-avl_devices.rules already exists"
    fi

)
# End subshell, return to user directory
echo "avl_sim_setup.sh: copied .config files to /var/avl_config/# for each simulated vehicle"
