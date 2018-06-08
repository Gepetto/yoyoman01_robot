#!/usr/bin/env python
import roslaunch
import rospy

rospy.init_node('tester', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid,["/home/ntestar/catkin_ws/src/yoyoman01_robot/yoyoman01_traject/launch/barriere.launch"])
launch.start()

rospy.spin()
