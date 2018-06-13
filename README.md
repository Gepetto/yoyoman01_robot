# yoyoman01_robot
Repository collecting all the ROS packages for the Yoyoman01 passive walker.

Gazebo simulation
=================
roslaunch yoyoman1_gazebo yoyoman01_world.launch

Rviz visualization
==================

roslaunch yoyoman01_description yoyoman01_rviz.launch

![alt text](https://raw.githubusercontent.com/Gepetto/yoyoman01_robot/master/yoyoman01_description/doc/YoyomanTheFirst.png)

Roscontrol tests
================
First launch the simulation:
roslaunch yoyoman1_gazebo yoyoman01_world.launch

Then starts the control framework:
roslaunch yoyoman1_control yoyoman01_control.launch

The starts the rqt perspective:
roslaunch yoyoman1_control yoyoman01_rqt.launch

Simulation
================
First put your movement and time matrices in a python code. Then copy the end of position_moteur.py to your code.

Run your code and replace in yoyoman01_motions.yaml and yoyoman01_gazebo.launch the previous values with your values. 

Finally launch yoyoman01_simulation on your shell.
