#!/bin/bash
#===============================================================================
# Autonomous Vehicle Library
#
# Description: This bash script lists the git status of all respoitories in the
#              avl folder.
#===============================================================================

# Cyan and no color escape characters
CYAN='\033[0;36m'
GOLD='\033[0;33m'
NC='\033[0m'

echo -e "${CYAN}===================================================================================${NC}"

# Get the path to this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Create a subshell to change the working directory to the avl/src directory
# search for all *.config files, and copy them to  /var/avl_logs/current/config
(

    # CD into the avl/src directory. This assumes that this script is located in
    # the avl/src/avl_tools/scripts directory
    cd "${DIR}/../../"

    # Run the find command to find all directories and run git status in them
    find . \
        -maxdepth 1 \
        -mindepth 1 \
        -type d \
        -exec sh -c '(echo "'${GOLD}'{}'${NC}'" &&
                      cd {} &&
                      git -c color.status=always status &&
                      echo &&
                      echo "'${CYAN}'==================================================================================='${NC}'")' \;

)
