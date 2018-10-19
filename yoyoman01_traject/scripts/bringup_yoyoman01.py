#!/usr/bin/env python
import os
import rospy
import roslaunch

if __name__ == "__main__":
  rospy.sleep(2)
  
  #launch controller
  rospy.init_node('tester', anonymous=True)
  uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
  roslaunch.configure_logging(uuid)
  launch = roslaunch.parent.ROSLaunchParent(uuid,["/home/up-board/Documents/catkin_ws/src/yoyoman01_robot/yoyoman01_traject/launch/yoyoman01_gazebo.launch"])
  launch.start()
  
  rospy.spin()
