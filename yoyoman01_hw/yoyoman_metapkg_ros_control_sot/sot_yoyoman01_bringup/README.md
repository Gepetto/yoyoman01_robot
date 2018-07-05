Launching geometric_simu with Talos
===================================

1/ Lancer ros et le simulateur geometric
roslaunch sot_pyrene_bringup geometric_simu.launch

2/ Lancer la connection a dynamic_graph_bridge
rosrun dynamic_graph_bridge run_command

3/ Construire son graphe dans la commande

4/ Demarrer la commande
rosservice call /start_dynamic_graph

5/ Lancer rviz
rosrun rviz rviz
