#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic
import os
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import Imu

def imu(data):
	rospy.loginfo("\n linear_acceleration: \n")
	rospy.loginfo("\n %s \n",data.linear_acceleration)
	longueur=len(str(data.linear_acceleration))
	compte=0
	x,y,z="","",""
	stop=False
	while stop==False:
		if str(data.linear_acceleration)[compte]=="y":
			stop=True
		elif str(data.linear_acceleration)[compte]!="x" and str(data.linear_acceleration)[compte]!=":":
			x=x+str(data.linear_acceleration)[compte]
		compte=compte+1
	stop=False
	
	while stop==False:
		if str(data.linear_acceleration)[compte]=="z":
			stop=True
		elif str(data.linear_acceleration)[compte]!="y" and str(data.linear_acceleration)[compte]!=":":
			y=y+str(data.linear_acceleration)[compte]
		compte=compte+1
	
	while compte<longueur:
		if str(data.linear_acceleration)[compte]!="z" and str(data.linear_acceleration)[compte]!=":":
			z=z+str(data.linear_acceleration)[compte]
		compte=compte+1
	rospy.loginfo(x+y+z)
	rospy.sleep(1) 

def get_link(data):
    rospy.loginfo("\n \n \n \n \n \n \n \n %s \n",data.name)
    
    #pose HeadLink
    rospy.loginfo("\n %s \n",data.pose[6])
    rospy.sleep(1)

def get_joint(data):
	rospy.loginfo("\n %s \n",data.name)
	rospy.loginfo("%s \n",data.position)
	data_float=[float(data.position[0]),float(data.position[1]),float(data.position[2]),float(data.position[3]),float(data.position[4]),float(data.position[5])]
	rospy.loginfo("\n %f %f %f %f %f %f ", data_float[0], data_float[1], data_float[2], data_float[3], data_float[4], data_float[5])
	rospy.sleep(1) 


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/gazebo/link_states', LinkStates, get_link)
    rospy.Subscriber('/joint_states', JointState, get_joint)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.Subscriber('/imu', Imu, imu)  
    rospy.spin()

if __name__ == '__main__':
    listener()
