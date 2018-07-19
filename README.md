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

First dowload sot-romeo from stack-of-tasks, then remove the src file in it and put the sot-yoyoman01'src file. Follow the readme instruction of the package to install it on your cpu.

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

/!\ /!\ /!\ You may have path problems, you can abuse of export in your bashrc file /!\ /!\ /!\ 

Aide pour FCam1
================
I) Play_motion
Tout se passe dans le paquet yoyoman01_traject. Le launch yoyoman01_simulation.launch va te lancer la simulation gazebo du robot avec les mouvements rentrés. Pour le robot réel il faut utiliser yoyoman01_real_robot_play_motion.launch.

Dans ce dernier launch, on lance tout d'abord les contrôleurs en appelant yoyoman01_control2.launch, celui-ci a besoin pour fonctionner d'abord de la description (l'urdf du robot) notament pour savoir quel type d'interface (position, vitesse, effort) il va rencontrer avec quel type de contrôle (cela sert aussi à lancer la visualisation sur Rviz après). On appelle ensuite joint_trajectory_controllers.yaml (du fichier config) qui lui spécifie le type de controller, le joint_state_controller et le fullbody_controller de type position_controllers/JointTrajectoryController ( type de controller propre à play_motion). Ce fichier yaml permet aussi de spécifier l'erreur permise sur la position et le temps d'atteinte de cette position des joints pour la trajectoire ainsi que les gains des joints.

On a ensuite un gros bloc moveit dans le fichier yoyoman01_real_robot_play_motion.launch (qui fait quasiment tout le fichier launch, on purrait le mettre dans un autre fichier launch qu'on appellerait ensuite), ce paquet permet d'envoyer la commande au controller en fonction de l'état des joints et de la trjaectoire voulu par play_motion.

à la fin de yoyoman01_real_robot_play_motion.launch, on appelle le fichier launch play_motion.launch, dans ce fichier on a d'abord une description srdf du robot (qui est relativement vide), celle-ci était demandé à un moment alors je l'ai mise mais je ne sais pas si elle est indispensable. On appelle ensuite en paramètre yoyoman01_motions3.yaml dans le dossier config, ce fichier spécifie les points que doivent atteindre les joints à des temps précis ainsi que le nom du mouvement auquel correspond ce mouvement. On appelle aussi approach_planner.yaml qui permet d'indiquer les controller utilisé et les joints non utilisés ainsi que d'autres paramètres dont je ne connais pas précisément l'utilité mais avec des valeurs identiques à celles vu sur d'autres robots. Ce fichier launch permet aussi de lancer le serveur play_motion

Enfin pour terminer on appelle un script python movement_real.py qui est le client pour play_motion et qui permet donc de demander le mouvement.

Dans yoyoman01_simulation.launch j'appelle beaucoup de script python qui me permette surtout de lancer les un après les autres les différents fichier launch cité au dessus ainsi que le gazebo et quelques commandes de position initial du robot dans gazebo.

Le script python (dans le dossier script) position_moteur4.py permet de faciliter la saisie d'un certain nombre de chose. Il faut lui rentré les matrices fourni par gabriele avec MUSCOD. Dans ce fichier il y a en premier les matrices correspondant à la phase d'initialisation du mouvement (avant qu'il marche vraiment ~régime transitoire), j'ai renommé ces matrices en rajoutant un indice "1" avec pour les différencier des autre matrices dans le code. Les autres matrices font référence au mouvement en régime permanent, elles correspondent à un pas. Ce que je fais alors dans mon script est de conacténé les matrices, en mettant la phase initiale en premier puis en ajoutant 10 fois la matrice d'un pas en symétrisant une fois sur deux (pour qu'il fasse le pied droit puis le gauche). On peut alors afficher (en lançant le script) tout d'abord les espaces de temps entre deux points (pour la matrice transitoire puis la matrice régime permanent), à partir de celles-ci tu peux choisir la fréquence de commande des points affiché avec les boucles de print qui suivent les opérations précédentes. Le code va ensuite t'afficher toutes positions voulues à mettre dans le fichier  yoyoman01_motions3.yaml (ou 4). (il permet aussi d'afficher d'autres informations importantes pour la position initiales dans la simulation).

