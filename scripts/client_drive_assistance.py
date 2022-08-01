#! /usr/bin/env python3

# Run with: roslaunch final_assignment assignment.launch

### LIBRARIES ###
import rospy
import curses

### SERVICE ###
from final_assignment.srv import Command

### CODE ###
def drive_assistance(ui):
    # Function to drive the robot througha controller that avoids hurting obstacles.

    # Creates a client to the command service.
    client = rospy.ServiceProxy("/command", Command)
    char = 'n'          # Sets the variable char to a default value.

    curses.noecho()     # From now on, the terminal does not print the pressed key.
    curses.cbreak()     # From now on, the input does not wait for the enter key.

    ui.set_wasd()       # Shows the commands for the free drive modality.
    ui.clear_info()

    # Waits until the command service is active.
    rospy.wait_for_service("/command")

    # Send the request of enabling the driver_assistance node.
    client(ord('0'))

    while True:

        char = ui.win_input.getch()   # Gets the user command.
        ui.clear_input()

        if char == ord('b'):   # Exits from the while loop.
            ui.clear_modes()  
            break

        # The character is one of the availables.
        elif char == ord('w') or char == ord('s') or char == ord('d') or char == ord('a') or char == ord('x') or char == ord('z'):
            # Send a request to the server with the user input character.
            resp = client(char)

            # Prints on the info window the linear speed.
            msg_linear = "Linear velocity: %.1f  " % resp.linear
            ui.win_info.addstr(2, 1, msg_linear)

            # Prints on the info window the angular speed.
            msg_angular = "Angular velocity: %.1f  " % resp.angular
            ui.win_info.addstr(3, 1, msg_angular)
            ui.win_info.refresh()

    # Stops the robot, sending a zero velocity.
    client(ord('x'))
    client(ord('z'))
    client(ord('1'))

    # Restores the default terminal paramethers.
    curses.echo()
    curses.nocbreak()

    # Prints the old avaliable commands.
    ui.clear_modes()
