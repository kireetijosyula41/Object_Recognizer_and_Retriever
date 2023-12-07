# Object Recognizer and Retriever
Team Members: Jake Bilbro, Jesse Gao, Tony Zhou, Kireeti Josyula

# Project Overview and Description: 

In daily life, there are daily tasks and applications that can be achieved by a robot, such as following a set of instructions to get and retrieve a certain set of items. Thus, in this final project, our group was interested to getting a turtlebot to move and navigate towards an intended object based on a user-input command, and having the user use their own hand to retrieve the items in question and return it to a user. As a extension of many parts of the Q-learning project, we hoped to achieve a robot program that could help people who are unable to move (e.g confined to bed, wheeelchairs, etc.), command a robot to identify, get and retrieve user-specified items. The two main components, thus, in this project, involved implementation of a custom Inverse Kinematics solver, as well as object classification and detection for a specified set of items that a turtlebot arm can manage. 

As shown in the two images below, our team has managed to implement a object detection algorithm that is able to recognize some of our preselected items (preselected items = ball, bottle, pen, cube), as shown in the screenshot of the turtlebot camera below. The second image shows the results of training our object detection algorithm on a subset of synthetic image data containing images of our four objects when placed against a background. The performance was high for the bottle and the ball and was moderate for the pen and the cube. 

Additionally, we managed to fine tune a custom inverse kinematics solver in order to get the robot arm to move to the motions of the hand through use of a depth camera. This functionality of this is displayed in the first GIF below:

Finally, in integrating both of these parts together, we have the GIF of the robot moving towards the intended target object (in the case below, the bottle), picking up the bottle through control with the hand, and finally returning to a pre-set location through use of an AR tag, displaying the full functionality of the project at hand. 


# System Architecture

There are four main files involved in this project. The names of these files, which are located in the scripts directory, are arm_control.py, hand_capture.py, object_detector.py







Main Components (1 paragraph): What are the main topics covered in your project? What robotics algorithm(s) will you be exploring in your project? Are there any other main components to your project?  
 
The most important part of this project will be the hand tracking in order to control the arm of the robot. In order to do this, we will need to implement hand tracking based on a depth camera, and use Inverse Kinematics in order to link hand movements to the movements of the robot arm. The next part of this project will be utilizing the robot’s locomotion capability so it can find the optimal path towards certain items. Our idea is to have the items labeled with AR tags and put in different locations in a closed room. This way, we will give the robot a command (either an auditory command or a typed-in command), and the robot will identify the location of the goal objects using a computer vision algorithm. Furthermore, the robot should be able to know its positions without us giving it such information. This way, if we can get it to learn its own location in space, we can have it navigate to and from objects and a set origin point. These steps may be implemented by using a particle filter localization algorithm while the robot moves autonomously, so that it can capture its position in space, navigate towards other objects, and carry out the previously-mentioned commands on picking up and returning objects to a set origin point. These should be the main components of the project. 

Final Product (1 bullet point per deliverable level, 2-3 sentences per bullet point): For each of the following, what would success look like:


MVP: For a minimally viable product, we hope to get the robot arm synced to the hand movements. The hope is that we can get the robot to move the arm according to different hand motions and positions. In addition, the minimally viable product should have the robot recognize and move to a certain location based on recognizing the AR tag and listening to the auditory command or other input command (typed) given. 

Solid End Result: A solid end result/goal is to have the robot be able to pick up an object based on our hand movements and then navigate back to a starting origin point based on the auditory or typed-in commands, while carrying that object. Thus, from start to finish, a solid end result should be a robot taking in a command, finding an appropriate AR tag location, navigating towards that location, picking up the object, and returning it to the user’s pre-set origin point


An end result that you would consider to be a "stretch" (e.g., "it would be above and beyond if we could..."): A ‘stretch’ ending would be to get the robot to deal with multiple different objects at a certain location (for example, if we needed a carrot and the AR tag location for carrot had both a carrot and tomato at the location, can it differentiate between the two objects and pick up the right one. This could be implemented with different colors, whereby the robot will pick up the right object based on a color for the object mentioned. 


Timeline: Provide a rough draft timeline for the major milestones of your project

Since we have 4 people, we will do vision(scene understanding, integrating GPT4, img frame/obj tracking) and hand tracking(integrating mediapipe and depth camera and IK) at the same time. This should take 1-1.5 week. After this we will merge and work on locomotion together. The two groups can work together at different times depending on our availability. The first group will work on the robot vision part to make the robot identify the correct locations of the goal objects. The second group will work on the hand tracking part so that the robot knows how to manipulate the objects.

Nov 10: Create a rough program framework like messages and launch files
Nov 13-14: The first group works on designing a basic moving program to allow tag identification. The second group works on basic stuff of hand gesture detection.
Nov 15-17: The first group tries to implement the object identification. The second group works on robot arm/gripper movements.
Nov 27-28: Build an experiment setting in reality and merge the results from the two groups together.
Nov 30-Dec 1: If it goes well, try to implement voice recognition and GPT-based instructions interpretation.
Dec 6 and 7: Having a working demo of the solid end result or ‘stretch’ depending on where we are, and all of the deliverables finished. 

Resources: Describe what materials you plan to use (e.g., additional sensors, objects for the robot to pick up, a maze environment) as well as details about the turtlebot(s) that you plan to use (how many turtlebots, whether or not you want the OpenManipulator arm(s) attached). 

For this project, we will need an in-depth camera in order to track hand movements and get the synced with the robot’s hand. We will be using the turtlebot’s camera and locomotion capability in order to get the robot to move towards certain locations. Finally, we will need objects and associated AR tags that the robot can recognize and pick up and retrieve to the set origin point. 


Risks (2-3 sentences): What do you see as the largest risks to your success in this project?

Some of the risks associated with this project that may hinder success will be integrating the voice commands or input commands into physical turtlebot movements. This is a major component of the project, so making sure the robot can respond appropriately to commands and respond appropriately will be key. The other risk for this project will be the in-depth camera for the hand motions, and figuring out how to correlate this to motions of the robot arm. 



