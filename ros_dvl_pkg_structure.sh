#!/bin/bash

# This script will create a standard ROS Doppler Velocity Logger Package

# Script requirements: 
# - ROS medolic
# - ROS standard workspace
# - git
# - pip 

# Process of creating the ROS DVL PKG:

cd ~/catkin_ws/src # Change directory to your source directory of your standard catkin workspace
catkin_create_pkg dvl rospy std_msgs message_generation message_runtime # create the dvl package
cd ~/catkin_ws # change directory to your standard catkin workspace
catkin_make # compile workspace 
. ~/catkin_ws/devel/setup.bash # Add the workspace to your ROS environment you need to source the generated setup file

# Standard package process setup:

cd ~/catkin_ws/src/dvl # change directory to the DVL pkg dir
touch README.md& # creates readme.md for the whole package directory
touch .gitignore

## Process of creating necessary directories for the package:

mkdir config& # used for .yaml files 
mkdir msg& # used for .msg files 
mkdir launch& # used for .launch files 
mkdir scripts # used for ros executable files 

## Process of creating initial files that are needed for this package:

## Setup of config Directory: 

cd config 
touch README.md # Creates readme.md for config directory

cd .. # change back 

## Setup of message Directory: 

cd msg
touch Raw_DVL.msg& # Creates .msg file for Raw DVL data
touch README.md # Creates readme.md for message directory

cd .. #change back 

## Setup of launch Directory: 

cd launch 
touch hydrus_dvl.launch& # Creates .launch file for DVL component
touch README.md # Creates readme.md for launch directory

cd .. # change back

## Setup of scripts Directory: 

cd scripts 
mkdir Client

### Setup of Client Directory:
 
cd Client
touch __init__.py& # this indicates that this directory will be a python package
touch dvl_client_node.py& # Creates the executable that the package will use
touch README.md # Creates readme.md for message directory

cd .. # change back
cd .. # change back

## Setup of Source Directory:

cd src 
touch README.md # Creates readme.md for source directory
mkdir lib # Directory that will hold libraries built by user or any programing library required.
cd lib 
touch README.md # Creates readme.md for lib directory
mkdir Server # Directory designated for anything that is related to the Server that will interface with the DVL 

### Setup of Server Directory: 

cd Server
touch __init__.py& # this indicates that this directory will be a python package
touch dvl_connector.py&
touch dvl_data.py& 
touch server.py&
touch requirements.txt&
touch start_serv&  
touch README.md # Creates readme.md for server directory
cd .. # change back

### Setup of Directory for the DVL Programming Library:
git submodule add https://github.com/Teledyne-Marine/Wayfinder
#git clone https://github.com/Teledyne-Marine/Wayfinder
cd Wayfinder 
pip3 install . # locally install python package

# Compile your catkin worskpace one last time:

cd ~/catkin_ws # change directory to your standard catkin workspace
catkin_make # compile workspace 

# Validation process:

cd ~/catkin_ws/src/dvl # change directory to the DVL pkg dir
rospack depends1 dvl # finally, check package dependacies 


# Note: The process of actually adding the libraries to the ros package so that the workspace recognizes it cannot be automated, but a protocol has also been created for to help with the task.
