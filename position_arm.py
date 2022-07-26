#!/usr/bin/env python
  
import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
  
class basic_subscriber:
  
    def __init__(self):
        # initialize the subscriber node now.
        # here we deal with messages of type Twist()
        self.image_sub = rospy.Subscriber("/robot/effort_joint_trajectory_controller/state", 
                                          JointTrajectoryControllerState, self.callback)
        print("Initializing the instance!")
  
    def callback(self, JointTrajectoryControllerState):
        
        # now simply display what
        # you've received from the topic
        ##rospy.loginfo(JointTrajectoryControllerState)
        print('Callback executed!')
        test = JointTrajectoryControllerState
	actual = test.actual.positions
	desired = test.desired.positions
	print (actual) 
	print (desired)
	print('GOOOO') 
	if  desired[0] - 0.05 <= actual[0] <= desired[0] + 0.05 and desired[1] - 0.11 <= actual[1] <= desired[1] + 0.11 and desired[2] - 0.11 <= actual[2] <= desired[2] + 0.11 and desired[3] - 0.11 <= actual[3] 		<= desired[3] + 0.11 and desired[4] - 0.11 <= actual[4] <= desired[4] + 0.11 and desired[5] - 0.11 <= actual[5] <= desired[5] + 0.11 and desired[6] - 0.15 <= actual[6] <= desired[6] + 0.15: 
		print ('PHASE I')
	 

  
def main():
    # create a subscriber instance
    sub = basic_subscriber()
     
    # follow it up with a no-brainer sequence check
    print('Currently in the main function...')
      
    # initializing the subscriber node
    rospy.init_node('listener', anonymous=True)
    rospy.spin()

  
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
