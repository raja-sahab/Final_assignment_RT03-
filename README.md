# Final_Assignment_RT03
## Task
>Develop a software architecture for the control of the robot in the environment. The software will rely on the move_base
and gmapping packages for localizing the robot and plan the motion.
The architecture should be able to get the user request, and let the robot execute one of the following behaviors
(depending on the user’s input):
 1) autonomously reach a x,y coordinate inserted by the user
 2) let the user drive the robot with the keyboard
 3) let the user drive the robot assisting them to avoid collisions.


## Installing and running

Run the program with:

```bash
$ git clone the respository
$ make all scripts executable from anywhere
$ roslaunch final_assignment assignment.launch
```

>For compiling and running, you need the `xterm` package; you can install it with:

```bash
$ sudo apt-get update
$ sudo apt-get install xterm
```

### Autonomous driving

>For autonomously driving the robot, the program sends a goal to the action server */move_base*, receiving feedbacks and monitoring the status until the goal is reached or canceled.
 Thanks to the *gmapping* algorithm, the robot can create a map of the surrounding environment during its tours. The `move_base` node implements the action server to control the robot through the shortest path to reach the given position.
 The user can also require to cancel the current goal or to send a new one.
 The `driving_mode` node asks the user to insert the goal with the `ask_for_goal` function.

### Free drive

>The function used to implement the free drive modality is in the `free_drive.py` file. It publishes the robot's speed on the topic */cmd_vel* each time the user inputs a command.

### Driver assistance

>The driver assistance modality uses the `driver_assistance` node, described in the second assignment, slightly changed to be turned on and off when it is used or not. For this reason, the code of the node is written in C++.
>The `client_drive_assistance.py` file contains the function to send a request to the `driver_assistance` node through the Command service, defined in the `srv` folder.

### User interface

>The user interface of the program is in `user_interface.py`. It is implemented with the *curses* library, dividing the terminal into windows and calling specific functions to print strings and receive user inputs.
 The class `windows_organiser` stores all information about the user interface and implements some modes to make the code slighter.
 
 ## Pseudocode
 
 ### driver_assistant
 
 ```
 function functionCallback:
   it is for the base_scan subscriber
   if
     is_active=true
     Copies the ranges values in the base_scan message into an array
     Calls the function scanSector to fill the sectors array
   else if
     function logic can take decision, call it
   else
     call function integral_logic

function scanSectors:
   to search for the closest obstacle in each sector
   If it finds a closest obstacle, than update the sectors array
   
function logic:
   It makes choices based on the values into the sectors array
   if
     front sector is free
     look for front-right and front-left sector to allign with path
   else if
     obstacle is on front
     then search left and right sectors respectively to find obstacle-free path and move accordingly
   else
     no sector is obstacle free then return 0
     
function integral_logic:
   Function to decide where to go when there are obstales all around the robot
   if
     right-area is greater than left area
     turn right
   else
     turn left

function integral
   Function to perform a discrete integral with the trapezium method
   while
      l is less than end
      use formula
      return result
      
function drive
   Function to drive the robot, 
   filling the geometry message and publishing it on the topic cmd_vel
   
function server_response
   it sets the speed of the motor depending on the message
   if
     input is 's' decrease linear-speed by 0.1
   else if 
     input is 'w' increase linear-speed by 0.1
   else if 
     input is 'x' reset linear-speed to 0.0
   else if 
     input is 'z' reset angular-speed to 0.0
   else if 
     input is 'd' decrease angular-speed by 0.1
   else if 
     input is 'a' increase angular-speed by 0.1
   else if
     input is '0' is_active=true
   else if
     input is '1' is_active=false
     
function main
   initialize driver_assistance node
   intialize values of global-variables
     speed=0.0
     d_br=1.2
   subscribe to the topic base_scan
   create a service for keyboard input to control the robot
   Creates a publisher to the cmd_vel topic for real-time controlling
   return 0
```

 ### autonomous_driving
 
 ```
 class autonomous_driving
    init
        Creates the move_action client

    function active_cb
        Increments the goal counter
        Prints info


    function feedback_cb
        Increments the feedback counter
        Prints info

    function done_cb
        Sets the active value to False
        Prints info about the returned value

    function reach_goal
        Sets the active value to True
        Waits until the action server has started
        Prints the new available commands
        Creates a goal to send to the action server
        Sends the goal to the action server

    function cancel_goal
        Sent a cancel goal to the action server
```

### client_drive_assistance

```
function drive_assistance
    Creates a client to the command service
    Waits until the command service is active
    Sends the request of enabling the driver_assistance node
    While True
        Gets the user command
        If the command is b
            Exits from the while loop;

        Else if the command is one of the available
            Sends a request to the server with the command;
            Prints on the info window the linear speed
            Prints on the info window the angular speed;

    Stops the robot
```

### driving_mode

```
function ask_for_goal

    Until the user does not insert a float

        Asks the user for the x position
        Gets the user input;

    Until the user does not insert a float

        Asks the user for the y position
        Gets the user input;

    Returns the coordinates of the goal

main

    Initialize the user interface
    Starts the ros node
    Initialize the autonomous driving

    While the roscore is active

        Gets the user command

        If the command is q, exit the while loop;

        Else if the command is 1, enters in the autonomous driving mode
            Calls ask_for_goal
            Sends a request to the server to reach the goal;

        Else if the command is 2, enters in the free drive mode
            If the autonomous drive is active, then cancel the goal
            Calls free_drive;

        Else if the command is 3, enters in the drive assistance mode
            If the autonomous drive is active, then cancel the goal
            Calls drive_assistance;

        Else if it is in the autonomous driving mode and the command is c
            Cancels the goal;

        Else if it is in the autonomous driving mode and the command is n
            Cancels the goal
            Calls ask_for_goal
            Sends a request to the server to reach the new goal;

        Else
            Print command not valid;
```

### free_drive

```
function free_drive
    Creates a publisher to the cmd_vel topic
    Shows the commands for the free drive modality
    While True
        Gets the user command
        If the command is b 
            Exits from the while loop;

        Else if the command is w
            Increases linear speed;

        Else if the command is s
            Decreases linear speed;

        Else if the command is d
            Turn right;

        Else if the command is a
            Turn left;

        Else if the command is x
            Stops linear speed;

        Else if the command is z
            Stops angular speed;

        Prints on the info window the linear speed
        Prints on the info window the angular speed
        Publishes the speed on the topic my_vel;

    Stops the robot
```

### user_interface
    
```
class windows_organiser
   init
	     Creates the win_modes wiindow for showing the available commands
	     Creates the win_info window for printing information
	     acquiring the user input and print it
		
   function clear_modes
	     Clears and resets the win_modes
		
   function clear_info
	     Clears and reset the win_info
		
   function clear_input
	     Clears the win_input
		
   function clear_request
	     Clears the win_request
		
   function set_wasd
	     Shows into the win_modes the command for directly drive the robot
		
   function command_not_valid
	     Sends the command not valid message into win_request
```

 ## Flow Chart
 
 ![Flow-chart (2)](https://user-images.githubusercontent.com/96690967/165001687-87b00af2-fd0d-4602-81b6-dd63f6d33416.jpg)

## Final Video

This is final video of autonomous drive

https://user-images.githubusercontent.com/96690967/165005263-c3eaea21-4cbc-4a29-b32f-b3b21900ef33.mp4

## Possible improvements

* Implements a modality to autonomously map all the environment.
* Implements an emergency stop service.
* Can calculate the reaching time.
* Can show error if the coordinates are outside of the map
