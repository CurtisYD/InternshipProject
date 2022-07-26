#! /usr/bin/env python
"""Publishes joint trajectory to move robot to given pose"""

import sys
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty
import argparse
import time

nbJoints = 7 
prefix = 'j2s7s300'
nbfingers = 3

def moveJoint (jointcmds,prefix,nbJoints):
  topic_name = '/robot/effort_joint_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  point.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0, nbJoints):
    jointCmd.joint_names.append('robot_j2s7s300_joint_'+str(i+1))
    point.positions.append(jointcmds[i])
    point.velocities.append(0)
    point.accelerations.append(0)
    point.effort.append(0) 
  jointCmd.points.append(point)
  rate = rospy.Rate(100)
  count = 0
  while (count < 500):
    pub.publish(jointCmd)
    count = count + 1
    rate.sleep()     

def moveFingers (jointcmds,prefix,nbJoints):
  topic_name = '/robot/effort_finger_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)  
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  point.time_from_start = rospy.Duration.from_sec(1.0)
  for i in range(0, nbJoints):
    jointCmd.joint_names.append('robot_j2s7s300_joint_finger_'+str(i+1))
    point.positions.append(jointcmds[i])
    point.velocities.append(0)
    point.accelerations.append(0)
    point.effort.append(0) 
  jointCmd.points.append(point)
  rate = rospy.Rate(100)
  count = 0
  while (count < 500):
    pub.publish(jointCmd)
    count = count + 1
    rate.sleep() 
  
def posarmarg():
	tab = []
	for i in range(2, len(sys.argv)):
		sa=sys.argv[i]
		a = float(sa)
		tab.append(a)
	return tab 

if __name__ == '__main__':
  try:    
    rospy.init_node('move_robot_using_trajectory_msg')		
   ## prefix, nbJoints, nbfingers = argumentParser(None)    
    #allow gazebo to launch
    time.sleep(5)

    # Unpause the physics
    rospy.wait_for_service('/gazebo/unpause_physics')
    unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    resp = unpause_gazebo()

    tab = posarmarg()
    

    print (tab) 
    moveJoint (tab,prefix,nbJoints)

    moveFingers ([0,0,0],prefix,nbfingers)
  except rospy.ROSInterruptException:
    print "program interrupted before completion"
