
mkdir –p src
catkin_make
source devel/setup.bash
cd ~/compsys726/src

>>>>>> Start
    >>>>>> Installation
    pip install numpy
    sudo apt install ros-melodic-pointcloud-to-laserscan

    >>>>>> To launch gazebo and turtlebot in random world
    roslaunch turtlebot_gazebo turtlebot_world.launch

    >>>>>> To launch rviz to see laserscanners and other properties
    roslaunch turtlebot_rviz_launchers view_robot.launch

    >>>>>> To launch keyboard controls to move turtlebot
    roslaunch turtlebot_teleop keyboard_teleop.launch