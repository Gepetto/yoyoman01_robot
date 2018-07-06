#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
import os
if __name__ == "__main__":
  unpause = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
  unpause()
  rospy.wait_for_service('gazebo/pause_physics')
  os.system('rosservice call gazebo/unpause_physics')
