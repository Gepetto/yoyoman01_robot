# yoyoman01_robot
Repository collecting all the ROS packages for the Yoyoman01 passive walker.
## ROS environment
You need : [ROS kinetic Desktop-Full](http://wiki.ros.org/kinetic/Installation/Ubuntu), playMotion, move-it, controller-manager, ros-controllers, gazebo-ros-control   

## Gazebo simulation

## Gazebo simulation
roslaunch yoyoman1_gazebo yoyoman01_world.launch

## Rviz visualization
roslaunch yoyoman01_description yoyoman01_rviz.launch

![alt text](https://raw.githubusercontent.com/Gepetto/yoyoman01_robot/master/yoyoman01_description/doc/YoyomanTheFirst.png)

## Roscontrol tests
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



Aide pour FCam1
================
## Lancer le test
```roscore```  
```rosrun yoyoman01_hw ros_hardware```  
```roslaunch yoyoman01_traject yoyoman01_real_robot_play_motion.launch```  

## Echantillonage trajectoire 
Il faut rentrer les matrices fourni par Gabriele dans le fichier : ```position_moteur41.py```(50hz)  situé dans ```/yoyoman01_robot/yoyoman01_traject/scripts/```. Dans ce fichier il y a en premier les matrices correspondant à la phase d'initialisation du mouvement (avant qu'il marche vraiment ~régime transitoire), j'ai renommé ces matrices en rajoutant un indice "1" (ex :t_v1). Les autres matrices font référence au mouvement en régime permanent, elles correspondent à un pas. Le script concatène les matrices, en mettant la phase initiale en premier puis en ajoutant x fois la matrice d'un pas en symétrisant une fois sur deux (pour qu'il fasse le pied droit puis le gauche)
  

Lancer le script : ```python3 position_moteur41.py```    
Le code va afficher toutes positions voulues.
Copier/coller dans le fichier  ```yoyoman01_motions41.yaml``` dans ```/yoyoman01_robot/yoyoman01_traject/config/```    

Modifier le fichier : ```play_motion.launch``` dans ```/yoyoman01_robot/yoyoman01_traject/launch/``` afin d'appeler ```yoyoman01_motions41.yaml```       

```play_motion.launch``` est appellé dans ```yoyoman01_real_robot_play_motion.launch``` 


I) Play_motion

Tout se passe dans le paquet yoyoman01_traject. Le launch yoyoman01_simulation.launch va te lancer la simulation gazebo du robot avec les mouvements rentrés. Pour le robot réel il faut utiliser yoyoman01_real_robot_play_motion.launch.

Dans ce dernier launch, on lance tout d'abord les contrôleurs en appelant yoyoman01_control2.launch, celui-ci a besoin pour fonctionner d'abord de la description (l'urdf du robot) notament pour savoir quel type d'interface (position, vitesse, effort) il va rencontrer avec quel type de contrôle (cela sert aussi à lancer la visualisation sur Rviz après). On appelle ensuite joint_trajectory_controllers.yaml (du fichier config) qui lui spécifie le type de controller, le joint_state_controller et le fullbody_controller de type position_controllers/JointTrajectoryController ( type de controller propre à play_motion). Ce fichier yaml permet aussi de spécifier l'erreur permise sur la position et le temps d'atteinte de cette position des joints pour la trajectoire ainsi que les gains des joints.

On a ensuite un gros bloc moveit dans le fichier yoyoman01_real_robot_play_motion.launch (qui fait quasiment tout le fichier launch, on purrait le mettre dans un autre fichier launch qu'on appellerait ensuite), ce paquet permet d'envoyer la commande au controller en fonction de l'état des joints et de la trjaectoire voulu par play_motion.

à la fin de yoyoman01_real_robot_play_motion.launch, on appelle le fichier launch play_motion.launch, dans ce fichier on a d'abord une description srdf du robot (qui est relativement vide), celle-ci était demandé à un moment alors je l'ai mise mais je ne sais pas si elle est indispensable. On appelle ensuite en paramètre yoyoman01_motions3.yaml dans le dossier config, ce fichier spécifie les points que doivent atteindre les joints à des temps précis ainsi que le nom du mouvement auquel correspond ce mouvement. On appelle aussi approach_planner.yaml qui permet d'indiquer les controller utilisé et les joints non utilisés ainsi que d'autres paramètres dont je ne connais pas précisément l'utilité mais avec des valeurs identiques à celles vu sur d'autres robots. Ce fichier launch permet aussi de lancer le serveur play_motion

Enfin pour terminer on appelle un script python movement_real.py qui est le client pour play_motion et qui permet donc de demander le mouvement.

Dans yoyoman01_simulation.launch j'appelle beaucoup de script python qui me permette surtout de lancer les un après les autres les différents fichier launch cité au dessus ainsi que le gazebo et quelques commandes de position initial du robot dans gazebo.

 

II) Stack of Tasks

Là ça va être moins détaillé, j'ai surtout repris des choses de pyrène.

Pour commencer: sot-yoyoman01, c'est un dossier à installer sur ton ordi, le fichier readme du dossier t'expliquera comment faire. ça crée surtout des bibliothèques pour les controllers et stack of tasks. Tout ce qui t'intéresse est dans src. sot-yoyoman01-device.cpp et sot-yoyoman01-device.hh sont là où tu auras à faire des modifs si besoin, notament au niveau des capteurs, je n'ai mis pour les joints que des capteurs d'angles (et non des capteurs d'angle moteur), mais tu peux modifier ça dans ces deux fichiers en décommentant les parties correspondantes, je te conseille d'explorer ces deux fichiers pour mieux comprendre (dans les grandes lignes tout du moins. Cela permet surtout de faire l'interface avec stack of tasks.

Toujours dans sot-yoyoman01, dans src/dynamic_graph/sot/yoyoman01, le fichier yoyoman01.py est important, il permet d'importer la représentation du robot dans la sot, le mieux c'est que tu explores à ta guise pour mieux comprendre. Le reste permet juste de tout mettre dans la sot.

Maintenant pour le paquet yoyoman01_sot_description, (qui lui es bien un paquet ros), il est surtout utilisé pour lancer la simulation gazebo avec le fichier launch. Mais du coup je pense qu'il faudra que tu crée un fichier launch pour ton robot dans lequel tu reprends les lignes de paramètres yoyoman01_world.launch (les rosparam et param), notamment celles concernant yoyoman_full_pal_hardware_gazebo_config.yaml et robot description (qui appelle des fichier yaml du dossier config du paquet), pid_gazebo.yaml devant être moins utiles que les autres, surtout présent pour gazebo. yoyoman_full_pal_hardware_gazebo_config.yaml sert juste à importer l'IMU dans stack of tasks, je ne sais pas si c'est surtout important pour gazebo ou si tu en auras aussi besoin. à voir. /!\ n'oublie pas d'importer l'argument control_mode pour choisir le type de controle (effort ou position) que tu veux utiliser.

Maintenant concernant le yoyoman_metapkg_ros_control_sot. Dans sot_yoyoman01_bringup on appelle les contrôleurs de la sot mais je sais pas si c'est que pour géometric simu donc va pas trop voir ça. Le plus intéressant cest roscontrol_sot_yoyoman, le launch que tu utilisera sera probablement sot_yoyoman_controller.launch (les autres c'est soit pour un control en effort soit pour une simu gazebo soit les deux), ce fichier launch te lance les contrôle en position sur le robot réel. Pour cela il appelle deux fichiers yaml qui se situe dans le config: sot_yoyoman_params.yaml qui appelle les bibliothèque de la sot créé et qui indique le type de controle utilisé et sot_yoyoman_controller.yaml qui appelle les contrôleurs et indique l'IMU.

Maintenant pour l'utiliser, si tu fais la simu tu lances en premier le launch yoyoman01_world.launch du paquet yoyoman01_sot_description, ensuite tu lances le sot_yoyoman_controller.launch du paquet roscontrol_sot_yoyoman, tes contrôleurs seront alors initialisés. Dans un autre shell lance alors "rosrun dynamic_graph_bridge run_command", ça te lance un shell python dans lequel travaillé, dans ce shell python tu peux alors commander la vitesse des joints du robot en lançant la commande robot.device.control.value=v avec v un vecteur de longueur 12 dans lequel les 6 premières valeurs correspondent à la position de la base (donc tu laisse à zéro car tu peux pas contrôler) et les 6 derniers aux vitesses initiales des joints (je ne sais plus l'ordre tu verras bien). Tu lances alors encore dans un autre shell "rosservice call /start_dynamic_graph" pour lancer dynamic graph et donc les commandes rentrées précédemment. 

/!\ dans robot.py de sot-yoyoman01 tu défini la position initiale half-sitting dans le même format que pour la commande (le vecteur v), tu peux donc modifier ça à ta guise pour choisir la position initiale lorsque tu lances les contrôleurs.

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
