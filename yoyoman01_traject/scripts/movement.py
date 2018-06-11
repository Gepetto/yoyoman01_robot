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
  rospy.sleep(0.1)
  #os.system('rosservice call gazebo/pause_physics')
  pause()
  rospy.loginfo("movement launch")
  goal = PlayMotionGoal()
  goal.motion_name = 'home'
  goal.skip_planning = True

  client.send_goal(goal)
  #os.system('rosservice call gazebo/unpause_physics')
  unpause()
  client.wait_for_result(rospy.Duration(10.0))
  rospy.loginfo("movement done.")
