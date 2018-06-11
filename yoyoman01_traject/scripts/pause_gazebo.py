#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
import os
if __name__ == "__main__":
  pause = rospy.ServiceProxy('gazebo/pause_physics', Empty)
  rospy.wait_for_service('gazebo/pause_physics')
  os.system('rosservice call gazebo/pause_physics')
