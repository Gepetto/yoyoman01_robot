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
You will only need yoyoman_traject

First put your movement and time matrices in a python code. Then copy the end of position_moteur.py to your code.

Run your code and replace in yoyoman01_motions.yaml and yoyoman01_gazebo.launch the previous values with your values. 

Finally launch yoyoman01_simulation on your shell.

Stack of Tasks
================
You will need "sot-yoyoman01", "yoyoman01_sot_description", "yoyoman_metapkg_ros_control_sot"

First dowload sot-roméo from stack-of-tasks, then remove the src file in it and put the sot-yoyoman01'src file. Follow the readme instruction of the package to install it on your cpu.

Secondly put yoyoman01_sot_description and yoyoman_metapkg_ros_control_sot in your catkin workspace before doing the catkin_make command.

You will need to install the following pkg:
ros-kinetic-hector-gazebo
robotpkg-pal-gazebo-plugins
robotpkg-pal-hardware-gazebo
robotpkg-pal-hardware-interfaces

and may be others that should be asked to you in the shell if they're missing.

You also may have to do some changes in the robotpkg-roscontrol-sot that you should download with the robotpkg package: The advice to use it are on:
http://robotpkg.openrobots.org/install.html
http://robotpkg.openrobots.org/robotpkg-wip.html (you will have to download wip in it)
In the roscontrol-sot package you will have to first download it the go in your work. directory created by the install to find sot-controllers.cpp to remove the forcetorquesensor and actuatortemperatursensor initialisation.

Then to launch the sot controllers:
in a first shell: roslaunch yoyoman01_sot_description yoyoman01_world.launch 
(you can modified control_mode in this launch file)
in a second shell: roslaunch roscontrol_sot_yoyoman sot_yoyoman_controller_gazebo.launch  (or sot_yoyoman_controller_gazebo_effort.launch if you modified it to an effort control)

/!\ Effort control mode hasn't been test yet

/!\/!\/!\ You may have path problems, you cans abuse of export in your bashrc file /!\/!\/!\ 
