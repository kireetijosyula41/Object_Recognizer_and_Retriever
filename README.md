# Object Recognizer and Retriever
Team Members: Jake Bilbro, Jesse Gao, Tony Zhou, Kireeti Josyula

# Project Overview and Description: 

In daily life, there are daily tasks and applications that can be achieved by a robot, such as following a set of instructions to get and retrieve a certain set of items. Thus, in this final project, our group was interested to getting a turtlebot to move and navigate towards an intended object based on a user-input command, and having the user use their own hand to retrieve the items in question and return it to a user. As a extension of many parts of the Q-learning project, we hoped to achieve a robot program that could help people who are unable to move (e.g confined to bed, wheeelchairs, etc.), command a robot to identify, get and retrieve user-specified items. The two main components, thus, in this project, involved implementation of a custom Inverse Kinematics solver, as well as object classification and detection for a specified set of items that a turtlebot arm can manage. 

As shown in the two images below, our team has managed to implement a object detection algorithm that is able to recognize some of our preselected items (preselected items = ball, bottle, pen, cube), as shown in the screenshot of the turtlebot camera below. The second image shows the results of training our object detection algorithm on a subset of synthetic image data containing images of our four objects when placed against a background. The performance was high for the bottle and the ball and was moderate for the pen and the cube, as shown in the imges below: 

Additionally, we managed to fine tune a custom inverse kinematics solver in order to get the robot arm to move to the motions of the hand through use of a depth camera. This functionality of this is displayed in the first GIF below:


# System Architecture

There are four main files involved in this project. The names of these files, which are located in the scripts directory, are arm_control.py, hand_capture.py, obj_dect.py and robot_driver.py. As mentioned above, the two main components for the lab were the gesture imitation and Inverse Kinematics linkage, as well as the Computer Vision algorithm for object detection. 

In tackling the computer vision part of the project, we wanted to implement an object classifier and detector for the four objects (bottle, pen, cube, ball), that we selected before. Initially, we created a synthetic image dataset, where isolated pictures of each of the objects were placed on top of a background image collected by the turtlebot camera. With this, we tried to use a pretrained IMAGE analysis and recgonition model called resnet_50, along with algorithmic from the following website which enabled road sign detection: 

After consulting with Sarah and talking with Teddy on how to improve the performance, we began to use YOLOv5, which was specifically intended for fast and quick object detection with accurate bounding boxes. Teddy provided us with a skeleton of a previous code iteration that worked with the Realsense camera.  In regards to the code, the file corresponding is obj_dect.py. In this file, a class ObjectDetection is called, which loads the YOLOv5 pretrained model, and returns the bounding boxes for all of the detected objects in the frame, along with the names of the objects. Within the umbrella function robot_driver.py, the code gathers this information from the obj_dect.py file in the image_callback() function, which uses the frame and the name of the object in tandem with a typed-in command ("Bottle", "Ball", "Cube", "Pen"). Through the view of a debugging window, it shows the object that was requested within a bounding box, which it then navigates towards through the cmd_vel topic with proportional control. 

# ROS Node Diagram

The following is the ROS node diagram for the complete framework of our code: 


ANNOTATE THIS WITH LIGHT COMMENTING ABOUT THE PROCESS

# Execution

The following instructions detail exactly how to insert the commands for complete execution of the task described above: 

1. Open a terminal window and run `roscore`
2. Bringup the turtlebot
In a new terminal window, ssh into the turtlebot and run `bringup`
In a new terminal window, ssh into the turtlebot and run `bringup_cam`
3. Prepare the arm to receive commands
In a new terminal window, run `roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch`
In a new terminal window, run `roslaunch turtlebot3_manipulation_moveit_config move_group.launch`
4. Republish the image data
In a new terminal window, run `rosrun image_transport republish compressed in:=raspicam_node/image raw out:=camera/rgb/image_raw`
5. Run the required nodes
In a new terminal window, run `rosrun Object_Organizer_and_Retriever hand_capture.py`
In a new terminal window, run `rosrun Object_Organizer_and_Retriever arm_control.py`
6. Finally, in a new terminal window, run `rosrun Object_Organizer_and_Retriever robot_driver.py`
Input commands “Bottle”, “Ball”, “Cube”, or “Pen” will command the robot to seek the respective object. The control of the arm will then be ceded to the user based on the depth camera. Once the object has been raised overhead (out of camera view), the robot will move to the AR tag.

Running these commands will enable the complete execution of the object recognition and retrieval program. 

# Challenges, Future Work, Takeaways

