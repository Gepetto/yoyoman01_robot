#!/usr/bin/env python
import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Empty
import os

if __name__ == "__main__":
  os.system('rosservice call gazebo/unpause_physics')
  rospy.init_node("home")
  rospy.loginfo("Waiting for play_motion...")
  client = actionlib.SimpleActionClient("/play_motion", PlayMotionAction)
  client.wait_for_server()
  rospy.loginfo("...connected.")

  rospy.wait_for_message("joint_states", JointState)
  ##pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
  rospy.sleep(0.1)
  #unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
  rospy.loginfo("home...")
  goal = PlayMotionGoal()
  goal.motion_name = 'home'
  goal.skip_planning = True

  client.send_goal(goal)
  #pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
  client.wait_for_result(rospy.Duration(10.0))
  rospy.loginfo("Move done.")
