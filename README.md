# InternshipProject

Prerequisites : 

   Summit_xl:
   
    https://github.com/RobotnikAutomation/summit_xl_sim.git
    
    https://github.com/Kinovarobotics/kinova-ros.git
    
  Computer vision: 
  
    https://github.com/EscVM/OIDv4_ToolKit.git
    
    Download Darknet 
  
  
  
Kinova_command folder: Functions allowing to move the robot, to move its arm and to perform the tasks of opening a door and moving an object. 

move_object.py and open_door.py take as input a parameter which corresponds to the position of the object or the door. Position : x y z Orientation : x y z w (eg. rosrun kinova_control open_door.py 2 0 0 0 0 1)  

Results folder : Video of the results obtained using the files open_door.py and move_object.py 

Yolov4 folder : Configuration files for door and handle detection. 
This command line allows to generate the images on which to work : python main.py downloader --classes '*Name_of_model*' --type_csv train --limit 1500 (*number_of_image_wanted*)
convert_label.py allows to give the right format to the labels of the images generated previously. 

#########################################################################################################################################################

Command to run programm: 

Yolov4: 

##Run custom detector with this command
HANDLE
Image detector : 
./darknet detector test Yolov4/obj.data Yolov4/yolov4-obj.cfg /Yolov4weights/yolov4-obj_last.weights Yolov4/image/handle/12.jpg -thresh 0.3

Video detector : 
!./darknet detector demo test Yolov4/obj.data Yolov4/yolov4-obj.cfg weights/yolov4-obj_last.weights -dont_show /image/test1.mp4 -i 0 -out_filename /mydrive/image/results.avi

Camera detector : 
./darknet detector demo Yolov4/obj.data Yolov4/yolov4-obj.cfg Yolov4/weights/yolov4-obj_last.weights -c 0



SUMMIT_XL : 

roslaunch summit_xl_sim_bringup summit_xl_complete.launch default_xacro:=summit_xl_gen_std.urdf.xacro launch_arm_a:=true arm_manufacturer_a:=kinova arm_model_a:=j2s7s300 amcl_and_mapserver_a:=false move_base_robot_a:=false

Moveit summit_kinova : 
ROS_NAMESPACE=robot roslaunch summit_xl_j2s7s300_moveit_config demo.launch

Python script to command robot : 

rosrun kinova_command move_arm.py 0.0 2.355 0.0 3.925 0.0 3.14 1.57

rosrun kinova_command move_robot.py 

rosrun kinova_command open_door.py 3 0 0 0 0 0 1 

90° = 1.57 ; 180° = 3.14 ; 360° = 6.28

#########################################################################################################################################################

 
