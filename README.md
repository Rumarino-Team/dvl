# Rumarino ROS Doppler Velocity Logger Package

## Installation

### ROS Installation
You must already have a catkin workspace to be able to add the package into the src folder.

In this example the root folder is named catkin_ws (it can have another name, does not have to be the same, but keep in mind to replace the your root folder name whenever the folder catkin_ws is used)

![Default catkin workspace](utils/images//readme_1.jpg)

The default catkin workspace will usually have three folders, build, devel, src.

<br/>
Go into the src directory and perform git clone.

  ```
  cd src
  git clone https://github.com/Rumarino-Team/dvl
  ```

![Git clone package](utils/images/readme_2.jpg)

Change back to the root of the catkin ws.

`cd ..`

Build the package with catkin make.

`catkin_make`

Source the built package in the terminal.

`source devel/setup.sh`

### Python2 Installation

Using Ubuntu 18.04.6 by default the command `python` and `pip` refers to the python2.7 version.

Install requirements for python2 with pip:

`pip install -r src/dvl/script/scripts/Client/requirements.txt`

### Python3 Installation
To install the components needed for python3 the packages `python3-pip` and `python3-venv`.

Use the command:

`sudo apt install python3-pip`<br/>
`sudo apt install python3-venv`

This package lets us create virtual environments for python3. In the root folder of the workspace create a new virtual environment with the following command:

`python3 -m venv <env_name>`

where <env_name> is the name of the environment. (It can have any name) In this example the virtual environment will be named `server_env`, to create the environment with this name the following command is used:

`python3 -m venv server_env`

Root workspace folder before creating virtual environment:

![Before venv create](utils/images/readme_3.jpg)

Root workspace folder after create virtual environment:

![After venv create](utils/images/readme_4.jpg)

Activate the virtual environment with:

`source server_env/bin/activate`

After activating the virtual environment the terminal should look something like this:

![After activating venv](utils/images/readme_5.jpg)


Install the python3 dependencies with pip into the virtual environment:

`pip install -r src/dvl/src/lib/Server/requirements.txt`


### Install Wayfinder library
The Teledyne DVL Wayfinder library is included in the repository. With the virtual environment active install the library using pip.

`pip install src/dvl/src/lib/Wayfinder`


## Running the Package
### Starting the python3 http server

Set the environment variable of flask to the python script that will run the server. (Assuming current working directory is root of catkin workspace) Open a new console and type the following commands.

`export FLASK_APP=src/dvl/src/lib/Server/server.py`<br/>
`flask run`

After these commands are entered and no errors occur in the console, the server can be verified with a browser going to the following url: `localhost:5000/dvl` and should see a response from the server.

__Example of server response in Firefox browser__

![Firefox Response Example](utils/images/readme_6.jpg)


### Starting the ros node server

> Make sure `roscore` is already running

With a new terminal make sure to source the ros melodic installation setup.bash file and source the workspace setup.bash file before trying to start the ros node or ros will not be able to find the packages.

__With the root of the catkin workspace as the current working directory sourcing the required files would look like:__

![Sourcing ROS Files](utils/images/readme_7.jpg)

To verify that ROS can see the package the command `rosrun` can be used by pressing __TAB__ key to see if autocomplete displays the package.<br/>
![Package in rosrun](utils/images/readme_8.jpg)

Then use the `rosrun` command with the name of the package and the name of the desired script to execute, in this case being the *dvl_client_node.py*:<br/>
`rosrun dvl dvl_client.py`

### Viewing the server and node connected for debugging
