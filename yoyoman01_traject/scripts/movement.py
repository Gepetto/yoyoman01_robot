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


  rospy.init_node("movement_node", anonymous=True)
  rospy.loginfo("Waiting for play_motion...")

  unpause()
  client = actionlib.SimpleActionClient("/play_motion", PlayMotionAction)

  pause()
  client.wait_for_server()
  rospy.loginfo("...connected.")

  unpause()
  rospy.wait_for_message("joint_states", JointState)
  rospy.sleep(0.09)
  pause()
  now = rospy.get_rostime()
  rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
  
  #set initial conditions before launch simulation
  #os.system("rosservice call /gazebo/set_model_state '{model_state: { model_name: yoyoman01, pose: { position: { x: 0.116245021398, y: -0.0763049851059, z: 0.815279089915 }, orientation: { x: 0.0497451000883, y: -0.0331579906013, z: -0.0327788139959, w: 0.998209016642 } }, twist: { linear: {x : -0.186742570216, y: 0.0402938445798, z : -0.00844364927522}, angular: {x: 0.76056627175555, y: 0.5703256717500523, z: 0.5760695316148732} } , reference_frame: world } }'")
  os.system("rosservice call /gazebo/set_model_state '{model_state: { model_name: yoyoman01, pose: { position: { x: -0.0476828472125, y: 0.0783765063995, z: 0.812800482088 }, orientation: { x: 0.00723480511602, y: -0.0332692134492, z: -0.0327405659257, w: 0.998358264766 } }, twist: { linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0} } , reference_frame: world } }'")
  rospy.rostime.wallsleep(2)

  
  #launch the movement
  rospy.loginfo("movement launch")
  goal = PlayMotionGoal()
  goal.motion_name = 'home'
  goal.skip_planning = True

 


  client.send_goal(goal)
  unpause()

  


  rospy.loginfo("movement done.")
  

