#!/usr/bin/env python
import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Empty
import os

if __name__ == "__main__":
  pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
  unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
  rospy.sleep(6)
  rospy.init_node("movement_node")
  rospy.loginfo("Waiting for play_motion...")
  #os.system('rosservice call gazebo/unpause_physics')
  unpause()
  client = actionlib.SimpleActionClient("/play_motion", PlayMotionAction)
  #os.system('rosservice call gazebo/pause_physics')
  pause()
  client.wait_for_server()
  rospy.loginfo("...connected.")
  #os.system('rosservice call gazebo/unpause_physics')
  unpause()
  rospy.wait_for_message("joint_states", JointState)
  rospy.sleep(0.09)
  #os.system('rosservice call gazebo/pause_physics')
  pause()
  rospy.loginfo("movement launch")
  goal = PlayMotionGoal()
  goal.motion_name = 'home'
  goal.skip_planning = True

  client.send_goal(goal)
  #os.system('rosservice call gazebo/unpause_physics')
  unpause()
  #os.system("rosservice call /gazebo/set_model_state 'model_state: \n model_name: 'yoyoman01'\n pose: \n  position: \n   x: 0.116245021398 \n   y: -0.0763049851059 \n   z: 0.815279089915 \n  orientation: \n   x: -0.00764170480825 \n   y: -0.0660306919426 \n   z: 0.0993339335946 \n   w: 0.0 \n twist: \n  linear: \n   x: 0.0 \n   y: 0.0 \n   z: 100.0 \n  angular: \n   x: 0.0 \n   y: 0.0 \n   z: 0.0 \n reference_frame: '''")
  #os.system("rosservice call /gazebo/set_model_state 'model_state: \n model_name: 'yoyoman01'\n pose: \n  position: \n   x: 0.116245021398 \n   y: -0.0763049851059 \n   z: 1.815279089915 \n  orientation: \n   x:0.0993339335946  \n   y: -0.0660306919426 \n   z: -0.00764170480825\n   w: 0.0 \n twist: \n  linear: \n   x: 0.18674257021624127 \n   y: 0.04029384457976738 \n   z: -0.008443649275220176 \n  angular: \n   x: 0.00025308740690237875 \n   y: 0.1973145837733334 \n   z: -0.051075066793250946 \n reference_frame: '''")
  client.wait_for_result(rospy.Duration(10.0))
  rospy.loginfo("movement done.")

