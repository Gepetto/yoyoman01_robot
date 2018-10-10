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
  rospy.loginfo("Waiting for server")

  # Waits until the action server has started up and started
  # listening for goals.
  client.wait_for_server()
  rospy.loginfo("...connected.")

  rospy.wait_for_message("joint_states", JointState)
  rospy.loginfo("joint_states get")
  rospy.sleep(1)

  #launch the movement
  rospy.loginfo("movement launch")
  goal = PlayMotionGoal()
  goal.motion_name = 'home'
  goal.skip_planning = True


  # Sends the goal to the action server.
  rospy.loginfo("Starting in 5s")
  rospy.sleep(1)
  rospy.loginfo("---------4")
  rospy.sleep(1)
  rospy.loginfo("-------3")
  rospy.sleep(1)
  rospy.loginfo("-----2")
  rospy.sleep(1)
  rospy.loginfo("--1")
  rospy.sleep(1)
  rospy.loginfo("GO!")

  #client.send_goal(goal)
  client.send_goal_and_wait(goal)
  rob_state = client.get_state 	()

  # Wait the end of the motion
  rospy.loginfo("movement done.")


  #while not rospy.is_shutdown():
    #i=0
  exit()

    