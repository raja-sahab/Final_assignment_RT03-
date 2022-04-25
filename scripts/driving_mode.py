#!/usr/bin/env python3

# Run with: roslaunch final_assignment assignment.launch

### STANDARD LIBRARIES ###
import rospy
import curses


### MY CODE ###
import user_interface
import autonomous_driving
import free_drive
import client_drive_assistance

### FUNCTION ###
def ask_for_goal():
    # This function asks the user to input the x and the y position of the goal, and returns them as floating point.
    while True:

        # Asks the user for the x position.
        ui.win_request.addstr(0, 0, "Insert the x position:")
        ui.win_request.refresh()

        # Gets the user input.
        x = ui.win_input.getstr(0, 0, 3)
        ui.clear_input()

        try:      # If the input can be converted into a float, it exits from the while.
            fx = float(x)
            break

        except:   # Asks again for the x position.
            ui.win_request.addstr(0, 23, "(try again)")

    ui.clear_request()

    while True:

        # Asks the user for the y position.
        ui.win_request.addstr(0, 0, "Insert the y position: ")
        ui.win_request.refresh()

        # Gets the user input.
        y = ui.win_input.getstr(0, 0, 3)
        ui.clear_input()

        try:      # If the input can be converted into a float, it exits from the while.
            fy = float(y)
            break

        except:   # Asks again for the y position.
            ui.win_request.addstr(0, 23, "(try again)")

    ui.clear_request()

    return fx, fy   # Returns the coordinates of the goal.

### MAIN ###
if __name__=="__main__":

    try:
        # The class windows_organiser is defined in the script user_interface.py
        # All the user interface is controlled with this class.
        ui = user_interface.windows_organiser()

        # Starts the ros node.
        rospy.init_node('driving_mode')

        # The autonomous_driving class is defined in the script autonomous_driving.py
        # The autonomous drive modality is controlled with this class.
        ad = autonomous_driving.autonomous_driving(ui)

        while not rospy.is_shutdown():

            key = ui.win_input.getch()      # Gets the user command.

            ui.win_info.addch(1, 18, key)   # Shows the command received.
            ui.win_info.refresh()

            if key == ord('q'):   # If the command is q, exit the while loop.
                break

            # If the command is 1, enters in the autonomous driving mode.
            elif key == ord('1') and ad.is_active is False:

                # Shows that it is in the autonomous driving mode.
                ui.clear_request()
                ui.win_modes.addstr(0, 0, "-->")
                ui.win_modes.refresh()

                x, y = ask_for_goal()   # Asks the user for a goal.
                ad.reach_goal(x, y)     # Sends a request to the server to reach the goal.

            # If the command is 2, enters in the free drive mode.
            elif key == ord('2'):

                # If the autonomous drive is active, then cancel the goal.
                if ad.is_active is True:
                    ad.cancel_goal()

                ui.clear_request()

                # Calls the function free_drive, which is defined in the script free_drive.py to enters the free drive mode.
                free_drive.free_drive(ui)

            # If the command is 3, enters in the drive assistance mode.
            elif key == ord('3'):

                # If the autonomous drive is active, then cancel the goal.
                if ad.is_active is True:
                    ad.cancel_goal()

                ui.clear_request()

                # Calls the function drive_assistance, which is defined in the script client_drive_assistance.py to enters the drive assistance mode.
                client_drive_assistance.drive_assistance(ui)
                

            # When it is in the autonomous driving mode, if c is pressed, cancels the goal.
            elif key == ord('c') and ad.is_active is True:
                ad.cancel_goal()        # Sends a cancel request.

            # When it is in the autonomous driving mode, if n is pressed, send a new goal.
            elif key == ord('n') and ad.is_active is True:
                ad.cancel_goal()        # Sends a cancel request.
                x, y = ask_for_goal()   # Asks for a new goal.
                ad.reach_goal(x, y)     # Sends a request to the server to reach the new goal.

            else:   # The command is not valid.
                ui.command_not_valid()

    except Exception as e:
        print(e)

    finally:
        curses.nocbreak()
        curses.echo()
        curses.endwin()
