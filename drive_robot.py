#! /usr/bin/env python
"""Publishes joint trajectory to move robot to given pose"""

import sys
import rospy
import rospkg
import math
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
import argparse
import time

def moveRobot(pos) : 
	topic_name = '/robot/move_base_simple/goal'
	pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
	pose = PoseStamped()
	pose.header.stamp = rospy.Time.now()
	pose.header.frame_id = "robot_odom"
	pose.pose.position.x = pos[0]
	pose.pose.position.y = pos[1]
	pose.pose.position.z = pos[2]
	
	pose.pose.orientation.x = pos[3]
	pose.pose.orientation.y = pos[4]
	pose.pose.orientation.z = pos[5]
	pose.pose.orientation.w = pos[6]

	desired = []
	desired.append(pose.pose.position.x)
	desired.append(pose.pose.position.y)
	desired.append(pose.pose.position.z)

	desired.append(pose.pose.orientation.x)
	desired.append(pose.pose.orientation.y)
	desired.append(pose.pose.orientation.z)
	desired.append(pose.pose.orientation.w)
	
	rate = rospy.Rate(100)
 	count = 0
	while (count < 50):
		pub.publish(pose)
		count = count + 1
		rate.sleep()
	return desired
##########################################################################################################
def posrobotarg():
	coord = []
	for i in range(1, len(sys.argv)):
		sa=sys.argv[i]
		a = float(sa)
		coord.append(a)
	return coord
##########################################################################################################
def position(coord): 
	
	where = coord 
	if coord[6]==1:
		x = coord[0] - 0.7 
		where[0] = x
	return where

##########################################################################################################
def convertquater(quaternions):
	
	q0 = quaternions[6] 
	q1 = quaternions[3]
	q2 = quaternions[4]
	q3 = quaternions[5]

	'''c1 = 2 * (q0*q2 + q2*q3)
	c2 = 1 - 2*(q1*q1 + q2*q2)
	x = math.atan2(c1,c2)
	
	c3 = 2*(q0*q2 - q3*q1)
	y = math.asin(c3)'''

	c4 = 2*(q0*q3 + q1*q2) 
	c5 = 1 - 2*(q2*q2 + q3*q3)
	z = math.atan2(c4,c5) 
 
	
	x1 = quaternions[0]
	y1 = quaternions[1]

	x2 = (x1 - 0.7 * math.cos(z)) 
	y2 = (y1 - 0.7 * math.sin(z)) 
	
	quaternions[0]= x2 
	quaternions[1]= y2 
	
	return quaternions 
##########################################################################################################
def findangle(angle): 

	q0 = angle[6] 
	q1 = angle[3]
	q2 = angle[4]
	q3 = angle[5]

	c4 = 2*(q0*q3 + q1*q2) 
	c5 = 1 - 2*(q2*q2 + q3*q3)
	z = math.atan2(c4,c5) 

	return z		
##########################################################################################################
def behind(x1y1,alpha): 
	x1 = x1y1[0]
	y1 = x1y1[1]
	
	x2 = (x1 + 2 * math.cos(alpha)) 
	y2 = (y1 + 2 * math.sin(alpha))

	x1y1[0] = x2
	x1y1[1] = y2

	return  x1y1
##########################################################################################################
if __name__ == '__main__':
  try:    
    rospy.init_node('move_robot_using_trajectory_msg')		
   ## prefix, nbJoints, nbfingers = argumentParser(None)    
    #allow gazebo to launch
    time.sleep(1)

    # Unpause the physics
    rospy.wait_for_service('/gazebo/unpause_physics')
    unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    resp = unpause_gazebo()
   
    tab = posrobotarg()
    moveRobot(tab)
    


  except rospy.ROSInterruptException:
    print ("program interrupted before completion")
