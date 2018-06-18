#!/usr/bin/env python

import os
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import Imu
import numpy as np
import math


rospy.init_node('listener', anonymous=True)
    
Q_angle=0.01
Q_gyro=0.0003
R_angle=0.01

x_bias=0
y_bias=0
z_bias=0

XP=[[0, 0],[0, 0]]
YP=[[0, 0],[0, 0]]
ZP=[[0, 0],[0, 0]]

KFangleX=0.0
KFangleY=0.0
KFangleZ=0.0

G_GAIN=0.070*180/np.pi
A_GAIN=0.0573

now = rospy.get_rostime()
Time= float(now.secs)+float(now.nsecs)*(10**(-9))
memory=int(Time)

AccAngle=[0.0,0.0,0.0]
GyrAngle=[0.0,0.0,0.0]

def set_angles(Ax, Ay, Az, Gx, Gy, Gz, DT):
	global A_GAIN
	global G_GAIN
	global AccAngle
	global GyrAngle

	rate_gyr_x = Gx*G_GAIN;
	rate_gyr_y = Gy*G_GAIN;
	rate_gyr_z = Gz*G_GAIN;
	
	GyrAngle[0]=GyrAngle[0]+rate_gyr_x*DT
	GyrAngle[1]=GyrAngle[1]+rate_gyr_y*DT
	GyrAngle[2]=GyrAngle[2]+rate_gyr_z*DT
	
	acc_x=A_GAIN*Ax
	acc_y=A_GAIN*Ay
	acc_z=A_GAIN*(Az+9.81)
	
	AccAngle[0]=np.arctan2(acc_y,acc_z)
	AccAngle[1]=np.arctan2(acc_z,acc_x)
	AccAngle[2]=np.arctan2(acc_x,acc_y)

def kalmanFilterX ( accAngle, gyroRate, DT):
	x=0.0
	S=0.0

	global KFangleX
	global Q_angle
	global Q_gyro
	global x_bias
	global XP
	KFangleX = KFangleX + DT * (gyroRate - x_bias)
 
	XP[0][0] = XP[0][0] + ( - DT * (XP[1][0] + XP[0][1]) + Q_angle * DT )
	XP[0][1] = XP[0][1] + ( - DT * XP[1][1] )
	XP[1][0] = XP[1][0] + ( - DT * XP[1][1] )
	XP[1][1] = XP[1][1] + ( + Q_gyro * DT )
 
	x = accAngle - KFangleX
	S = XP[0][0] + R_angle
	K_0 = XP[0][0] / S
	K_1 = XP[1][0] / S
 
	KFangleX = KFangleX + ( K_0 * x )
	x_bias = x_bias + ( K_1 * x )

	XP[0][0] = XP[0][0] - ( K_0 * XP[0][0] )
	XP[0][1] = XP[0][1] - ( K_0 * XP[0][1] )
	XP[1][0] = XP[1][0] - ( K_1 * XP[0][0] )
	XP[1][1] = XP[1][1] - ( K_1 * XP[0][1] )
 
	return KFangleX

def kalmanFilterY ( accAngle, gyroRate, DT):
	y=0.0
	S=0.0

	global KFangleY
	global Q_angle
	global Q_gyro
	global y_bias
	global YP
	KFangleY = KFangleY + DT * (gyroRate - y_bias)
 
	YP[0][0] = YP[0][0] + ( - DT * (YP[1][0] + YP[0][1]) + Q_angle * DT )
	YP[0][1] = YP[0][1] + ( - DT * YP[1][1] )
	YP[1][0] = YP[1][0] + ( - DT * YP[1][1] )
	YP[1][1] = YP[1][1] + ( + Q_gyro * DT )
 
	y = accAngle - KFangleY
	S = YP[0][0] + R_angle
	K_0 = YP[0][0] / S
	K_1 = YP[1][0] / S
 
	KFangleY = KFangleY + ( K_0 * y )
	y_bias = y_bias + ( K_1 * y )

	YP[0][0] = YP[0][0] - ( K_0 * YP[0][0] )
	YP[0][1] = YP[0][1] - ( K_0 * YP[0][1] )
	YP[1][0] = YP[1][0] - ( K_1 * YP[0][0] )
	YP[1][1] = YP[1][1] - ( K_1 * YP[0][1] )
 
	return KFangleY
	
def kalmanFilterZ ( accAngle, gyroRate, DT):
	z=0.0
	S=0.0

	global KFangleZ
	global Q_angle
	global Q_gyro
	global z_bias
	global ZP
	KFangleZ = KFangleZ + DT * (gyroRate - z_bias)
 
	ZP[0][0] = ZP[0][0] + ( - DT * (ZP[1][0] + ZP[0][1]) + Q_angle * DT )
	ZP[0][1] = ZP[0][1] + ( - DT * ZP[1][1] )
	ZP[1][0] = ZP[1][0] + ( - DT * ZP[1][1] )
	ZP[1][1] = ZP[1][1] + ( + Q_gyro * DT )
 
	z = accAngle - KFangleZ
	S = ZP[0][0] + R_angle
	K_0 = ZP[0][0] / S
	K_1 = ZP[1][0] / S
 
	KFangleZ = KFangleZ + ( K_0 * z )
	z_bias = z_bias + ( K_1 * z )

	ZP[0][0] = ZP[0][0] - ( K_0 * ZP[0][0] )
	ZP[0][1] = ZP[0][1] - ( K_0 * ZP[0][1] )
	ZP[1][0] = ZP[1][0] - ( K_1 * ZP[0][0] )
	ZP[1][1] = ZP[1][1] - ( K_1 * ZP[0][1] )
 
	return KFangleZ









def imu(data):
	global Time
	global memory
	#rospy.loginfo("\n linear_acceleration: \n")
	#rospy.loginfo("\n %s \n",data.linear_acceleration)
	longueur=len(str(data.linear_acceleration))
	compte=0
	Ax,Ay,Az="","",""
	stop=False
	while stop==False:
		if str(data.linear_acceleration)[compte]=="y":
			stop=True
		elif str(data.linear_acceleration)[compte]!="x" and str(data.linear_acceleration)[compte]!=":":
			Ax=Ax+str(data.linear_acceleration)[compte]
		compte=compte+1
	stop=False
	
	while stop==False:
		if str(data.linear_acceleration)[compte]=="z":
			stop=True
		elif str(data.linear_acceleration)[compte]!="y" and str(data.linear_acceleration)[compte]!=":":
			Ay=Ay+str(data.linear_acceleration)[compte]
		compte=compte+1
	
	while compte<longueur:
		if str(data.linear_acceleration)[compte]!="z" and str(data.linear_acceleration)[compte]!=":":
			Az=Az+str(data.linear_acceleration)[compte]
		compte=compte+1
	#rospy.loginfo(Ax+Ay+Az)
	
	
	Gx,Gy,Gz="","",""
	compte=0
	stop=False
	while stop==False:
		if str(data.angular_velocity)[compte]=="y":
			stop=True
		elif str(data.angular_velocity)[compte]!="x" and str(data.angular_velocity)[compte]!=":":
			Gx=Gx+str(data.angular_velocity)[compte]
		compte=compte+1
	stop=False
	
	while stop==False:
		if str(data.angular_velocity)[compte]=="z":
			stop=True
		elif str(data.angular_velocity)[compte]!="y" and str(data.angular_velocity)[compte]!=":":
			Gy=Gy+str(data.angular_velocity)[compte]
		compte=compte+1
	
	while compte<longueur:
		if str(data.angular_velocity)[compte]!="z" and str(data.angular_velocity)[compte]!=":":
			Gz=Gz+str(data.angular_velocity)[compte]
		compte=compte+1
	#rospy.loginfo(Gx+Gy+Gz)
	
	now = rospy.get_rostime()
	DT= float(now.secs)+float(now.nsecs)*(10**(-9))-Time
	Time= float(now.secs)+float(now.nsecs)*(10**(-9))
	
	Ax, Ay, Az, Gx, Gy, Gz= float(Ax), float(Ay), float(Az), float(Gx), float(Gy), float(Gz)
	set_angles(Ax, Ay, Az, Gx, Gy, Gz, DT)
	angleX=kalmanFilterX(AccAngle[0],GyrAngle[0],DT)
	angleY=kalmanFilterY(AccAngle[1],GyrAngle[1],DT)
	angleZ=kalmanFilterY(AccAngle[2],GyrAngle[2],DT)
	#print("\n \n \n X="+str(angleX)+" Y="+str(angleY)+" Z="+str(angleZ)+" \n \n")
	if Time>memory+1:
		memory=int(Time)
		rospy.loginfo(str(Ax)+" "+str(Ay)+" "+str(Az))
		rospy.loginfo(str(Gx)+" "+str(Gy)+" "+str(Gz))
		print("\n \n \n X="+str(angleX*180/np.pi)+" Y="+str(angleY*180/np.pi)+"\n \n")
	rospy.sleep(0.0001) 

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
    #rospy.Subscriber('/gazebo/link_states', LinkStates, get_link)
    #rospy.Subscriber('/joint_states', JointState, get_joint)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.Subscriber('/imu', Imu, imu)  
    rospy.spin()

if __name__ == '__main__':
    listener()
