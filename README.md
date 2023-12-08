# Object Recognizer and Retriever
Team Members: Jake Bilbro, Jesse Gao, Tony Zhou, Kireeti Josyula

# Project Overview and Description: 

In daily life, there are daily tasks and applications that can be achieved by a robot, such as following a set of instructions to get and retrieve a certain set of items. Thus, in this final project, our group was interested to getting a turtlebot to move and navigate towards an intended object based on a user-input command, and having the user use their own hand to retrieve the items in question and return it to a user. As a extension of many parts of the Q-learning project, the interesting part of this project that we aimed to achieve a robot program that could help people who are unable to move (e.g confined to bed, wheeelchairs, etc.), command a robot to identify, get and retrieve user-specified items. The two main components, thus, in this project, involved implementation of a custom Inverse Kinematics solver, as well as object classification and detection for a specified set of items that a turtlebot arm can manage. 

As shown in the two images below, our team has managed to implement a object detection algorithm that is able to recognize some of our preselected items (preselected items = ball, bottle, pen, cube), as shown in the screenshot of the turtlebot camera below. The second image shows the results of training our object detection algorithm on a subset of synthetic image data containing images of our four objects when placed against a background. The performance was high for the bottle and the ball and was moderate for the pen and the cube, as shown in the imges below: 
![](https://github.com/kireetijosyula41/Object_Recognizer_and_Retriever/blob/main/val_batch0_labels.jpg)
![](https://github.com/kireetijosyula41/Object_Recognizer_and_Retriever/blob/main/val_batch0_pred.jpg)

Additionally, we managed to fine tune a custom inverse kinematics solver in order to get the robot arm to move to the motions of the hand through use of a depth camera. This functionality of this is displayed in the first GIF below:

![](https://github.com/kireetijosyula41/Object_Recognizer_and_Retriever/blob/main/HandTracking.gif)

Integrating all components was accomplished by the following procedure: First, the robot identifies the object specified by the user and moves to it. Then, the control of the arm is ceded to the user. Once the user has picked up the object (gripper closed for >10s) the robot returns to its original positions, finds the correct AR tag, and places the object in front of it. The entire process can be seen in the GIF below, indicating completion of the process that we hoped to achieve above: 

![](https://github.com/kireetijosyula41/Object_Recognizer_and_Retriever/blob/main/ORARMain.gif)

# System Architecture

There are four main files involved in this project. The names of these files, which are located in the scripts directory, are arm_control.py, hand_capture.py, obj_dect.py and robot_driver.py. As mentioned above, the two main components for the lab were the gesture imitation and Inverse Kinematics linkage, as well as the Computer Vision algorithm for object detection. 

## Computer Vision
In tackling the computer vision / object detection part of the project, we wanted to implement an object classifier and detector for the four objects (bottle, pen, cube, ball), that we selected before. Initially, we created a synthetic image dataset, where isolated pictures of each of the objects were placed on top of a background image collected by the turtlebot camera. With this, we tried to use a pretrained IMAGE analysis and recgonition model called resnet_50, along with algorithmic from the following website which enabled road sign detection: https://towardsdatascience.com/bounding-box-prediction-from-scratch-using-pytorch-a8525da51ddc, which we cite here. However, we were running into many problems with this procedure, which included failure to recognize specific objects, failure to properly capture the bounding box of the object in view, and significant lag when using the turtlebot camera to identify images. The model performs much worse in bounding box detection than object classification. Indeed, training a regression model to output 4 values by feeding hundreds of thousands of pixels into it seems a challenging task, given the huge space of exploration to find the correlation between the two.

After consulting with Sarah and talking with Teddy on how to improve the performance, we began to use YOLOv5 rather than Resnet_50, which was specifically intended for fast and quick object detection with accurate bounding boxes. Teddy provided us with a skeleton of a previous code iteration that worked with the Realsense camera, which we adapted to work with the turtlebot camera parameters.  In regards to the code for this process, the file corresponding to this is obj_dect.py. In this file, a class ObjectDetection is called, which loads the YOLOv5 pretrained model, and returns the bounding boxes for all of the detected objects in the frame, along with the names of the objects. Within the umbrella function robot_driver.py, the code gathers this information from the obj_dect.py file in the image_callback() function, which uses the frame and the name of the object in tandem with a typed-in command ("Bottle", "Ball", "Cube", "Pen"). Through the view of a debugging window, it shows the object that was requested within a bounding box, which it then navigates towards through the cmd_vel topic with proportional control. 

## Inverse Kinematics
The implementation of inverse kinematics is handled by the `arm_control.py` node. The node will receive hand data from the `hand_control_topic` topic and attempt to move the end-effector of the arm to this position. Since the robot arm will almost exclusively be reaching forward, the position of the arm is locked to the front half of the robot (y > 0). Additionally, it is okay to use only the arm-down solutions, as the primary goal is to reach for objects on the ground in front of the robot.

This was achieved by separating the arm into 4 sections.

Joint 1:
* The first joint rotates the entire arm about the `z` (up) axis, and is calculated using basic trigonometry based on `x` and `y`

Joints 2 and 3:
* The second and third joints are treated as a 2DOF arm, with the target position being some point in the y-z plane. This significantly reduced the complexity of the inverse kinematics problem, and is solved through the `two_RIK()` function

Joint 4:
* The final joint is mapped to the tilt_angle received from the hand camera. It should intuitively mimic the angle of the user’s wrist. This value does not need to be adjusted.

Gripper:
* The gripper is set to be opened or closed as a function of the distance between the user’s thumb and pointer finger. When the user pinches their fingers together, the gripper will close. When the user separates their fingers, the gripper will open. The gripper value is scaled and set in `get_hand_pos()`

Algorithm:
The algorithm is only active when a value of “Start” is received from the `robot_state`. Otherwise, target joint angles are not published
Once the hand data is received, the `y` and `z` coordinates must be converted from camera-space to world-space. If the point `(y, z)` exceeds the maximum distance the arm can reach, the coordinates will become `(y’, z’)` the nearest point on the circle.
![](https://github.com/kireetijosyula41/Object_Recognizer_and_Retriever/blob/main/circle_diagram.jpg)
After adjusting the points, the 2D Kinematics Algorithm runs and produces a set of joint angles. However, these angles are not from the same reference points, and several adjustments must be made for each angle before the command is issued.


# ROS Node Diagram

The following is the ROS node diagram for the complete framework of our code: 
![](https://github.com/kireetijosyula41/Object_Recognizer_and_Retriever/blob/main/OOARArchitectureDiagram.jpg)

`robot_driver.py`: takes input from the user in the form of a string command. It then published that information to the `robot_state` topic. It also calls the relevant object detection functions from `obj_dect`.

`hand_capture.py`: publishes all necessary hand-tracking data to the `hand_control_topic` topic. 

`arm_control.py`: Subscribes to the `robot_state` and `hand_control_topic` topics. Based on the state value received, the IK algorithm will be activated or deactivated.

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

# Challenges
## Inverse Kinematics
The inverse kinematics component was more challenging than expected. The first issue we ran into was the discrepancy between the angles calculated and the angles used by moveit. 
The angles for joint1 produced by the algorithm are relative to the `y` (front) axis, but the required angle is relative to the `z` (up) axis. The angles for joint2 produced by the algorithm are relative to the axis produced along the first link, but the required angle is relative to 90 degrees of this axis. Once these conversions were made, the algorithm now produces the expected joint angle targets

There were many issues in practice with the real turtlebot. The time it takes for moveit to plan and execute the trajectory results in latencies between 100ms and 1000ms. This makes it slightly more difficult for the user to precisely control the arm. There were also bizarre behaviors during the procedure. At seemingly random times, even though the hand position data smoothly moves between points, the arm will stop moving and then suddenly jerk to the desired position at an incredible rate of speed. To mitigate this, we placed speed and acceleration limits on all the joints. We also limited the domain to the front half of the robot to try to limit the distance between desired points. For the most part, this behavior no longer happens. The reason as to why this issue exists is still not fully understood, but adjusting the rate and the speed limits seems to have helped.

## Hand Tracking
The depth-camera presented multiple challenges to our project. It was difficult to work on this aspect because the camera requires drivers to run that had to be installed on the linux machine by techstaff. The drivers were only installed on Jesse’s account on the “Arcanine” machine, meaning Jesse had to be present and we had to work on that specific computer. 
At first, the camera data was noisy and inaccurate, and struggled to detect the hand properly. Z-values were consistently jumping around and only worked in a small area. This caused excessive jitteriness and snapping movements of the arm, making the inverse kinematics aspect of the project more difficult to implement. Once the cause of the issue was diagnosed and fixed, most of these problems disappeared.

## Object Detection
The objection detection component can be very frustrating. We underestimated the difficulty of object detection tasks in comparison to object classification. Our original choice resnet_50 is typically designed for image classification. Though it may be adapted to produce bounding boxes, we can only make it output 4 values to represent the 4 coordinates of a single bounding box. It is infeasible to get multiple bounding boxes. We thought of ways to cope with this disadvantage such as masking the detected objects that are not our target so that the rest of the objects can be detected, but the model simply does not perform very well in detecting the boundary box. Fine-tuning both the two pretrained models resnet and YOLOv5 can be time-consuming if we want to have better results with larger dataset and larger training images. It also took us some time to figure out the right modifications on the training parameters, where we found two helpful improvements: greater weight decay, larger image size, and smaller batch size. The synthetic datasets also seem ineffective if the running environment does not match the background or the object images are not realistic enough. But they are needed due to the need for bounding box designation.

# Future Work
The inverse kinematics algorithm could potentially be refined. If we had more time, we would look into ways to reduce the latency on the robot movements, as inexperienced users may have a difficult time adjusting to this. Additionally, the remaining jerky movement behaviors could potentially undergo smoothing. For example, if the target position is more than 7.0cm away, split the movements into increments and perform each one consecutively. There is a lot of potential for improvements in the object detection model. If we have more time, we will fine tune it on larger datasets and image sizes with longer time. We can use Teddy's computer for more rapid computations in the training process with GPU.

# Takeaways
It is extremely important to test components of projects individually. A lot of time was spent debugging the inverse kinematics algorithm to no avail since the actual problem was the camera data being sent. If we had been more diligent in testing both the inverse kinematics and the hand tracking separately, we would have been much more efficient
Once each component was refined separately, the integration was fast and simple.

Object detection is a difficult process, but choosing the right model is important as well. At first we were using RestNet, which is primarily an image classifier. This presented issues with speed and accuracy. For the final version, we switched to YOLOv5, which is more commonly used for object detection. This improved the speed and allowed for us to successfully identify the objects. Had we selected a more appropriate model, time could have been saved which we would have used to refine the object detection component.
