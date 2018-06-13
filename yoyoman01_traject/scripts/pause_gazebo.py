#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
import os
if __name__ == "__main__":
  pause = rospy.ServiceProxy('gazebo/pause_physics', Empty)
  rospy.wait_for_service('gazebo/pause_physics')
  os.system('rosservice call gazebo/pause_physics')
  rospy.sleep(3)
  #set_state()=rospy.ServiceProxy("/gazebo/set_model_state", Empty)
  #set_state('{model_state: { model_name: yoyoman01, pose: { position: { x: 0.116245021398, y: -0.0763049851059 ,z: 0.815279089915 }, orientation: {x: -0.00764170480825, y: -0.0660306919426, z: 0.0993339335946, w: 100000000000 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }')
  #os.system("rosservice call /gazebo/set_model_state '{model_state: { model_name: yoyoman01, pose: { position: { x: 0.116245021398, y: -0.0763049851059 ,z: 0.815279089915 }, orientation: { x: 0.0497451000883, y: -0.0331579906013, z: -0.0327788139959, w: 0.998209016642 } }, twist: { linear: {x : 0.186742570216, y: 0.0402938445798, z : -0.00844364927522}, angular: {x: -0.12656298115932776, y: 0.8615789797231312, z: 0.5627992346857531} } , reference_frame: world } }'")
  ##set gravity
  #rospy.sleep(3)
  
  ###################################     Cette ligne permet de modifier la gravitée     ###################################  
  
                                 # /!\ pente à 0.05 radians dans le sens des x négatifs /!\#
  
  #os.system('gz physics -g 0,-0.4902956505,-9.7977400545')
  
  ##########################################################################################################################
  
  
  #os.system('rosservice call gazebo/unpause_physics')
  #os.system("rosservice call /gazebo/set_model_state 'model_state: \n model_name: 'yoyoman01'\n pose: \n  position: \n   x: 0.116245021398 \n   y: -0.0763049851059 \n   z: 0.815279089915 \n  orientation: \n   x: -0.00764170480825 \n   y: -0.0660306919426 \n   z: 0.0993339335946 \n   w: 0.0 \n twist: \n  linear: \n   x: 0.0 \n   y: 0.0 \n   z: 100.0 \n  angular: \n   x: 0.0 \n   y: 0.0 \n   z: 0.0 \n reference_frame: '''")
