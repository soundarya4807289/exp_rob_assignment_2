# Exp_Rob_Lab_ass2
Soundarya Pallanti assignment 2 repository

Introduction
--------------

This is the Assignment 2 of Experimental Robotics Laboratory.

The code simulates a "dog" robot which walks randomly on the environment and then goes to sleep if the user does not inputs a "play" command.
The play command can be input at any times and the robot will finish its movement and go to play.

When the "play" command is given the robot will finish its current action and then an interface will be displayed in order to select the position where the ball is desired to go, once this is done the robot starts searching for the ball using a camera module. The robot will locate the ball eventually and then starts approaching it, after it arrives, the program will ask for a new point to move the ball to, and the robot will "check its surroundings" before it starts searching for the ball once again. This will be repeated until the user inputs the command "stop".

When the command "stop" is introduced, the ball will disappear from the environment. The robot will then, start to look for some time the ball, and then go back to its normal behavior, waiting for another "play" command

Software Architecture and States Diagrams
----------------------------------------

For the software architecture:

![](https://github.com/soundarya4807289/exp_rob_assignment_2/blob/main/Assignment_2_architecture.png)

The architecture shows the working principle of the program. First the "command_request" node will be waiting for the user to input a command, in parallel the program and "state_machine" node will be running on NORMAL state witing also for said command. "state_machine" node is the brain of the program, on NORMAL or SLEEP state, will communicate with "move_robot_server" and with the robot model to move it to the random locations or sleep location respectively. Once it enters into the PLAY state (after 'play' command is received) the state machine will make use of "camera_ball" and "play_coords" nodes, to move the ball to the desired position and to start controlling the robot through the camera input. It will remain on PLAY state until at some point the 'stop' command is input, which will switch to NORMAL state once again.
  
For the State Machine Diagram:

![](https://github.com/soundarya4807289/exp_rob_assignment_2/blob/main/Assignment_2_architecture.png)

The State Machine has 3 states:
  - NORMAL: Main state in which the robot walks randomly until the "play" command arrives and goes to PLAY state, if not, goes to SLEEP state.
  - SLEEP: The robot goes to the sleep position and rests for a while, afterwards wakes up and goes to the NORMAL state.
  - PLAY: The robot plays until the "stop" command arrives locating the ball with the camera. When the playing stops goes to NORMAL state again.
  
The robot will switch between states depending on the commands received.
  - 'play' will switch from NORMAL to PLAY
  - 'stop' will switch from PLAY to NORMAL
  - 'wait' (not input by the user) will switch from NORMAL to SLEEP, and recursively from SLEEP to NORMAL
  
Messages
----------

The message types used in the project are:
  - geometry_msgs Point: for x,y coordinates of the robot.
  - geometry_msgs Pose: to get the pose of the ball and the robot
  - geometry_msgs Twist: for controlling ball and robot velocities.
  - navigation_msgs Odometry: to get the odometry data of the robot and ball.
  - sensor_msgs CompressedImage: to get and show the image using opencv.
  - std_msgs Float: to control the angle of the camera on the robot.
  - std_msgs String: for strings.
  - std_msgs Bool: for booleans.
  
Actions
-----------

The action used are located in the /action folder, it has the shape:
  - goal: geometry_msgs PoseStamped 
  - response: geomterty_msgs Pose 
  
This action message is used to move the robot on the NORMAL and SLEEP states, and also to move the ball on the PLAY state. Both uses the same message but are different actions as shown on the architecture.
  
Parameters
-------------

The parameter are to be specified just on the sm_assignment file:
  - (optional: commented on launch file) normal_times: Times to walk randomly on normal state (default random)
  - sleep_x: Bed x coordinate (between -8,8) (Default 7)
  - sleep_y: Bed y coordinate (between -8,8) (Default 1)
  - time_sleep: Time the robot sleeps (between 1-10) (Default random)
  - wait_time: Time for the robot to wait while "checking surroundings" -> checks one side, waits, checks other side, waits, centers again.
  
Packages and File list
------------
- Launch:
The launch file is gazebo_world.launch located inside /launch folder. The created nodes are at the end of the file, with their respective parameters.

- Nodes:
All the executable files are in /scripts folder which are the .py files for each node.

  - camera_ball: Node in charge of controlling the camera installed on the robot. Once it enters in the play state, it starts searching on the image for a green ball, after if founds it, starts approaching to it and when the robot arrives to the ball, starts "checking its surroundings"
  
  - command_request: Node in charge of reading the inputs of the user, "play" or "stop" are the valid commands.
  
  - go_to_point_ball: Node in charge of moving the ball to the specified coordinates, it's the server of the /reacihng_goal action.
  
  - move_robot_server: Node in charge of moving the robot to the random coordinates on the environment while in NORMAL state of to go to sleep on the SLEEP state. It's the server of the /move_goal action.
  
  - play_coords: Node in charge of displaying the grid so the user can select the coordinates to which the ball will move. It's the client of /reaching_goal action.
  
  - state_machine: Node in charge of managing the state machine of the program. Will switch through states and maganes the main behavoir of the robot. It's also the client of /move_goal acton.


- The documentation can be found in the folder : /docs
In order to open it on firefox, open a terminal inside /docs folder and run: 
  - firefox _build/html/index.html
  
- The robot structure files can be found inside /urdf folder under the ex3_arm.xacro and ex3_arm.gazebo name files, which are the ones that define the structure of the robot, and its controlled joint.

- For the control file is can be found inside /config as ex3_motors.yaml.

- The /world folder contains the world definition, this file hasn't been modified after its delivery.

Intstallation and Running Procedure
-----------------

Download the project, or clone the repository in the desired folder on your computer.

Use: 
- roslaunch exp_assignment2 gazebo_world.launch

It will launch all the required nodes and the program will be fully functional. parameters can be specified on the same command or inside the .launch file located on the /launch folder.

In order to display extra information on the terminal, all the codes have comented the respective lines which will display information like changes of status, distance to goal while on the camera_ball control, etc. The currently displayed information on the terminal is what it's considered for me the essential information. Current goal, when it arrives, when it finishes checking its surroundings, changes of state, and ball coordinates chosen (which to where the robot will approach using the camera when it sees the ball, but will never reach that position).

Working Hypothesis and Environment
-------------

For the working hypothesis the robot should start in a NORMAL state, for which will move randomly until the play command arrives or the amount of times finishes and goes to SLEEP, giving the image of the camera all the time. If it goes to SLEEP, afterwards re-enters on the NORMAL behavior. If a "play" command is sent during the random walks, the robot will finish its current walk and then enter the PLAY state even if he hasn't finished all the random walks. At this point, the ball will come onto the environment on the coordinates specified by the user and the robot will start tracking on the image for the ball and go towards it when it finds it. The robot will arrive (somewhat close to the ball, while trying to make the robot go closer, got into conflicts with the stability of the movement control), and check its environment, where the user will need to input a new coordinate to move the ball to. The behavior will repeat until the user inputs the "stop" command. 

While checking the surroundings, the image will freeze until this movement is done, then will refresh and the robot will keep searching for the ball on the image. In this unfreezing stage the image will get all of the previous frames (they weren't shown while checking the surroundings as it's a while loop and doesn't refreshes the image) so for a brief moment the robot might have abrupt reactions as will get movement of the ball and try to follow it real quickly. The mass of the frontal limb was increased in order to avoid big "willies" which can destabilize the robot.

When the "stop" command arrives, the ball will hide automatically (negative z coordinate) and it will be announced on the terminal. The robot will search for the ball during a small amount of time and then re-enter the NORMAL state.

When the robot goes to SLEEP, he must wake up first in order to enter the play state, if a command is sent during the sleeping time, he will wake up, go to NORMAL state and directly to PLAY state.

The environment is a 16x16 grid, with a man, a bed and the dog. The robot will constantly say the random walk iterations it's going to do, in which one is it, and towards which goal he is going. It will also display the input request on the terminal, which can be input a any time. Once this is given, and the coordinates are specified, will say towards which point the ball was moved, when the robot arrives to the ball, and when it finishes doing the "check surroundings". The robot never knows the ball coordinates, but these are displayed in order to know which ones where the ones selected on the interface. 

System's Features
------------
  - Commands can be input at any time.
  - Terminal shows the "usually" interesting data.
  - 16x16 display grid to select ball desired coordinates.
  
System's Limitations and technical improvements
------------

Robot can misbehave after "checking surroundings" due to the fast refreshing of the image if the ball comes too close to the camera, or through the robot, which will make the robot to try to follow it in an abrupt way. On the testings the robot even lost balance and falled down. The mass of the frontal limb was increased so it gets compensated to an extent. On the behavior testing after the mass was increased, this scenario didn't happen again, so the issue seems to not be fatal anymore.

Couldn't try for long periods of time the algorithm as it is a high computational demand on my computer and the simulation is not fast. But tested on all the states switching on the same run, going from NORMAL to SLEEP, to waking up and NORMAL again, into PLAY and back to NORMAL, etc.

Author and Contacts
------
Soundarya Pallanti
email: s4807289@studenti.unige.it