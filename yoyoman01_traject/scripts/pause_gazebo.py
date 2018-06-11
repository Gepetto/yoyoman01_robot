#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
import os
if __name__ == "__main__":
  pause = rospy.ServiceProxy('gazebo/pause_physics', Empty)
  rospy.wait_for_service('gazebo/pause_physics')
  os.system('rosservice call gazebo/pause_physics')
  ##set gravity
  #rospy.sleep(3)
  #os.system('gz physics -g 0,-0.4902956505,-9.7977400545')
  #os.system('rosservice call gazebo/unpause_physics')
  #os.system("rosservice call /gazebo/set_model_state 'model_state: \n model_name: 'yoyoman01'\n pose: \n  position: \n   x: 0.116245021398 \n   y: -0.0763049851059 \n   z: 0.815279089915 \n  orientation: \n   x: -0.00764170480825 \n   y: -0.0660306919426 \n   z: 0.0993339335946 \n   w: 0.0 \n twist: \n  linear: \n   x: 0.0 \n   y: 0.0 \n   z: 100.0 \n  angular: \n   x: 0.0 \n   y: 0.0 \n   z: 0.0 \n reference_frame: '''")
