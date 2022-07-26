#! /usr/bin/env python
import sys
import rospy
import rospkg
import argparse
import time
import math
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty
from std_msgs.msg import String
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

##########################################################################################################

##Move the joints of the arm by publishing a message to the topic
##Take in entry the position wanted of each joints

def moveJoint (jointcmds):
  topic_name = '/robot/effort_joint_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  point.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0, 7):
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
##########################################################################################################

##Move the fingers by publishing a message to the topic
##Take in entry the position wanted of each fingers

def moveFingers (jointcmds):
  topic_name = '/robot/effort_finger_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)  
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  point.time_from_start = rospy.Duration.from_sec(1.0)
  for i in range(0, 3):
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
##########################################################################################################
def callbacks(JointTrajectoryControllerState):
	
	global bypass2
	global bypass3
	global bypass4
	global tab1

      
        # Create an object JointTrajectoryControllerState
        armpos = JointTrajectoryControllerState
	#Get the actual position of each joints 
	actual = armpos.actual.positions
	#Get the desired position of each joints
	desired = armpos.desired.positions
	

	if  desired[0] - 0.05 <= actual[0] <= desired[0] + 0.05 and desired[1] - 0.11 <= actual[1] <= desired[1] + 0.11 and desired[2] - 0.11 <= actual[2] <= desired[2] + 0.11 and desired[3] - 0.11 <= actual[3] <= desired[3] + 0.11 and desired[4] - 0.11 <= actual[4] <= desired[4] + 0.11 and desired[5] - 0.11 <= actual[5] <= desired[5] + 0.11 and desired[6] - 0.15 <= actual[6] <= desired[6] + 0.15 and bypass2 ==1: 
		
		print ('PHASE II')
		moveJoint (arm_mo2)
		#Disable the loop
		bypass2 = 0
		#Enable the next loop
		bypass3 = 1 

	
	if  1 - 0.05 <= actual[0] <= 1 + 0.05 and desired[1] - 0.11 <= actual[1] <= desired[1] + 0.11 and desired[2] - 0.11 <= actual[2] <= desired[2] + 0.11 and desired[3] - 0.11 <= actual[3] <= desired[3] + 0.11 and desired[4] - 0.11 <= actual[4] <= desired[4] + 0.11 and desired[5] - 0.11 <= actual[5] <= desired[5] + 0.11 and desired[6] - 0.15 <= actual[6] <= desired[6] + 0.15 and bypass3 == 1:
		print('PHASE III') 
		moveJoint (arm_default)
		#Disable the loop
		bypass3 = 0
		#Enable the next loop
		bypass4 = 1

	if  0.0 - 0.15 <= actual[0] <= 0.0 + 0.15 and desired[1] - 0.2 <= actual[1] <= desired[1] + 0.2 and desired[2] - 0.2 <= actual[2] <= desired[2] + 0.2 and desired[3] - 0.2 <= actual[3] <= desired[3] + 0.11 and desired[4] - 0.2 <= actual[4] <= desired[4] + 0.2 and desired[5] - 0.2 <= actual[5] <= desired[5] + 0.2 and desired[6] - 0.2 <= actual[6] <= desired[6] + 0.2 and bypass4 == 1:
		print('PHASE IV') 
		alpha = findangle(tab1)
		secndpos = behind(tab1,alpha)
		moveRobot(secndpos)
		#Disable the loop		
		bypass4 = 0

##########################################################################################################

#Publisher : Move the robot by publishing on a topic 

def moveRobot(pos) : 
	topic_name = '/robot/move_base_simple/goal'
	pub = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
	#Create a PoseStamped object
	pose = PoseStamped()

	#Send the desired coordinate
	pose.header.stamp = rospy.Time.now()
	pose.header.frame_id = "robot_odom"
	pose.pose.position.x = pos[0]
	pose.pose.position.y = pos[1]
	pose.pose.position.z = pos[2]
	
	pose.pose.orientation.x = pos[3]
	pose.pose.orientation.y = pos[4]
	pose.pose.orientation.z = pos[5]
	pose.pose.orientation.w = pos[6]
	
	rate = rospy.Rate(150)
 	count = 0
	while (count < 50):
		pub.publish(pose)
		count = count + 1
		rate.sleep()
##########################################################################################################

# Subscriber : Allow us to receive data from a topic 
def callback(Odometry):


	global bypass
	global bypass2
	global coord
	
	test = Odometry
	actual = []
	px = test.pose.pose.position.x
	py = test.pose.pose.position.y	
	pz = test.pose.pose.position.z
		
	ox = test.pose.pose.orientation.x
	oy = test.pose.pose.orientation.y
	oz = test.pose.pose.orientation.z
	ow = test.pose.pose.orientation.w

	actual.append(px)
	actual.append(py)
	actual.append(pz)
	actual.append(ox)
	actual.append(oy)
	actual.append(oz)
	actual.append(ow) 
	desired = coord
	##print(actual)
	##print(desired)

	if  desired[0] - 0.15 <= actual[0] <= desired[0] + 0.15 and desired[1] - 0.11 <= actual[1] <= desired[1] + 0.11 and desired[2] - 0.11 <= actual[2] <= desired[2] + 0.11 and desired[3] - 0.11 <= actual[3] <= desired[3] + 0.11 and desired[4] - 0.11 <= actual[4] <= desired[4] + 0.11 and desired[5] - 0.11 <= actual[5] <= desired[5] + 0.11 and desired[6] - 0.15 <= actual[6] <= desired[6] + 0.15 and bypass == 1:
		print('PHASE I')
		moveJoint (arm_mo1)
		#Disable the loop
		bypass = 0
		#Enable the next loop
		bypass2 = 1
		

def checkpos():
	rospy.Subscriber("/robot/robotnik_base_control/odom", Odometry, callback)
	rospy.Subscriber("/robot/effort_joint_trajectory_controller/state", JointTrajectoryControllerState, callbacks)
	# simply keeps python from exiting until this node is stopped
	rospy.spin()
##########################################################################################################

#Take the argument of the python file and put them in a variable
def posrobotarg():
	tab = []
	for i in range(1, len(sys.argv)):
		sa=sys.argv[i]
		a = float(sa)
		tab.append(a)
	return tab

##########################################################################################################
def convertquater(quaternions):

	#Convert the quaternions angle in euler angle syste
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
 
	#Get the coordinate position in front of the door for the robot
	x1 = quaternions[0]
	y1 = quaternions[1]

	x2 = (x1 - 0.7 * math.cos(z)) 
	y2 = (y1 - 0.7 * math.sin(z)) 
	
	quaternions[0]= x2 
	quaternions[1]= y2 
	
	return quaternions 
##########################################################################################################
#Get the orientation
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
#get the coordinate behind the object
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

    arm_default = [0.0,3.14,0.0,1.57,0.0,3.14,0.0]
    arm_mo1 = [5.28,1,1.7,3.14,6,1.5,1.57]
    arm_mo2 = [1,1,1.7,3.14,6,1.5,1.57]
    
    #Enable/Disable some loop
    bypass = 1 
    bypass2 = 0
    bypass3 = 0
    bypass4 = 0

    #Initialisation of a node
    rospy.init_node('move_robot_using_trajectory_msg')		
    time.sleep(1)
    tab = posrobotarg()
    tab1 = posrobotarg()
    coord = convertquater(tab)
    print(coord)
    moveRobot(coord)       
    checkpos()
   
    

  except rospy.ROSInterruptException:
    print "program interrupted before completion"

