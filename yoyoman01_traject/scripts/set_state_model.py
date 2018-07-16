#!/usr/bin/env python
import rospy
import roslaunch
  
if __name__ == "__main__":
  rospy.sleep(6.5)
  uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
  roslaunch.configure_logging(uuid)
  launch = roslaunch.parent.ROSLaunchParent(uuid,["/home/ntestar/catkin_ws/src/yoyoman01_robot/yoyoman01_traject/launch/set_state.launch"])
  launch.start()
  rospy.spin()
