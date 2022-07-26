#!/usr/bin/env python
  
import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from nav_msgs.msg import Odometry



def callback(Odometry):
      
        # now simply display what
        # you've received from the topic
        ##rospy.loginfo(JointTrajectoryControllerState)
        print('Callback executed!')
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
	print (actual) 

def main():
      
    # initialize a node by the name 'listener'.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/robot/robotnik_base_control/odom", Odometry, callback)
      
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
  
  
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
