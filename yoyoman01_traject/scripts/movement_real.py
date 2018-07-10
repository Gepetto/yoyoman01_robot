#!/usr/bin/env python
import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Empty
import os






if __name__ == "__main__":

  rospy.sleep(6)


  rospy.init_node("movement_node", anonymous=True)
  rospy.loginfo("Waiting for play_motion...")


  client = actionlib.SimpleActionClient("/play_motion", PlayMotionAction)


  client.wait_for_server()
  rospy.loginfo("...connected.")


  rospy.wait_for_message("joint_states", JointState)
  rospy.sleep(1)


  
  #launch the movement
  rospy.loginfo("movement launch")
  goal = PlayMotionGoal()
  goal.motion_name = 'home'
  goal.skip_planning = True

 


  client.send_goal(goal)


  


  rospy.loginfo("movement done.")
  
