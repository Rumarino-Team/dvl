#!/usr/bin/env bash

# Script should be at the root of all the packages but for now it will be in root of dvl package
# Script is not unfinished, it does not work, don't use it.

current_dir=$(pwd)
echo $current_dir
# ros install source path
ROS_INSTALL_PATH=""
# ros development package source path
ROS_PACKAGE_PATH_CURRENT=""

# python virtual environment path
PYTHON_VIRT_ENV=""

# flask server install path
FLASK_SERVER_INSTALL_PATH=""

# flask main app
FLASK_APP=""

if [[ $USERNAME == "estol" ]]; then
    # set ros env
    ROS_INSTALL_PATH="/opt/ros/melodic/setup.bash"
    ROS_PACKAGE_PATH_CURRENT="/home/estol/Documents/RUMarinoTeamGit/ros_official_ws/devel/setup.bash"
    PYTHON_VIRT_ENV="/home/estol/Documents/RUMarinoTeamGit/ros_official_ws/src/dvl/server_env/bin/activate"
    FLASK_APP="/home/estol/Documents/RUMarinoTeamGit/ros_official_ws/src/dvl/src/lib/Server/server.py"    
    echo "Current user is $USERNAME, ros install path and ros package path are defined."        
    
elif [[ $USERNAME == "hydrus" ]]; then
    # implement
    echo "Not yet implemented."
else
    echo "Provide path to ros installation: "
    read ROS_INSTALL_PATH
    echo "Provide path to current ros development workspace: "
    read ROS_PACKAGE_PATH_CURRENT
fi

activate_virt_env() {
    source $PYTHON_VIRT_ENV
    echo "Python virtual environment is set."
}

source $ROS_INSTALL_PATH
source $ROS_PACKAGE_PATH_CURRENT

gnome-terminal -- sh -c "roscore; exec bash"

#activate_virt_env

flask run
