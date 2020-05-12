A readme.txt file that contains instructions on how to build and run the project.
Read overview before launching due to notes and assumptions made.

>>>>>> Installation: Run these in terminal to install packacges
sudo apt update
sudo apt install python3-pip
pip install numpy
pip install opencv-python
sudo apt install ros-melodic-pointcloud-to-laserscan

>>>>>> Create catkin workspace: run these in terminal in home directory
mkdir compsys726
cd compsys726
mkdir src
catkin_make

>>>>>> Copy the python_nodes folder into the src folder ~/compsys726/src/
>>>>>> Then run these in terminal
chmod +x ~/compsys726/src/python_nodes/src/movement.py
chmod +x ~/compsys726/src/python_nodes/src/takephoto.py
chmod +x ~/compsys726/src/python_nodes/src/statelog.py
chmod +x ~/compsys726/src/python_nodes/src/help.py

>>>>>> Navigate to the folder and source using these commands in terminal:
cd ~/compsys726/src
source ~/compsys726/devel/setup.bash

>>>>>> Start World: NOTE: I had the world files in a worlds folder which was located in the home directory
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$PWD/worlds/example_1.world
    >>> OR
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$PWD/worlds/example_2.world

>>>>>> Launch the implemented nodes
roslaunch python_nodes py.launch