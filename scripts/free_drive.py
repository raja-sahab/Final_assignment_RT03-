#! /usr/bin/env python3

# Run with: roslaunch final_assignment assignment.launch

### LIBRARIES ###
import rospy
import curses

### MESSAGE ###
from geometry_msgs.msg import Twist

### CODE ###
def free_drive(ui):
    # Function to drive directly the robot, controlling the speed publishing on the cmd_vel topic.

    # Creates a publisher to the cmd_vel topic.
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    char = 'n'          # Sets the variable char to a default value.
    straight = 0        # Sets the linear velocity to zero.
    turn = 0            # Sets the angular velocity to zero.
    my_vel = Twist();   # Creates a Twist message.

    curses.noecho()     # From now on, the terminal does not print the pressed key.
    curses.cbreak()     # From now on, the input does not wait for the enter key.

    ui.set_wasd()       # Shows the commands for the free drive modality.
    ui.clear_info()

    while True:

        char = ui.win_input.getch()   # Gets the user command.
        ui.clear_input()

        # Understands the user command.
        if char == ord('b'):     # Exits from the while loop.
            ui.clear_modes()
            break

        elif char == ord('w'):   # Increases linear speed.
            straight += 0.1

        elif char == ord('s'):   # Decreases linear speed.
            straight += -0.1

        elif char == ord('d'):   # Decreases ang. speed (right)
            turn += -0.1

        elif char == ord('a'):   # Increases ang. speed (left)
            turn += 0.1

        elif char == ord('x'):   # Stops linear speed
            straight = 0

        elif char == ord('z'):   # Stops angular speed
            turn = 0

        # Prints on the info window the linear speed.
        msg_linear = "Linear velocity: %.1f  " % straight
        ui.win_info.addstr(2, 1, msg_linear)

        # Prints on the info window the angular speed.
        msg_angular = "Angular velocity: %.1f  " % turn
        ui.win_info.addstr(3, 1, msg_angular)
        ui.win_info.refresh()

        # Publishes the speed on the topic my_vel.
        my_vel.linear.x = straight;
        my_vel.angular.z = turn;
        pub.publish(my_vel);

    # Stops the robot, sending a zero velocity.
    my_vel.linear.x = 0;
    my_vel.angular.z = 0;
    pub.publish(my_vel);

    # Restores the default terminal paramethers.
    curses.echo()
    curses.nocbreak()

    # Prints the old avaliable commands.
    ui.clear_modes()
