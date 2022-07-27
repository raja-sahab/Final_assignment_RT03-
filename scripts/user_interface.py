#! /usr/bin/env python3

''' Run with: roslaunch final_assignment assignment.launch '''

import curses   ''' Library for printing on the whole screen.'''
import text     ''' Script with some text. '''

''' CODE '''
class windows_organiser:
    # This class manages the user interface, dividing the terminal into windows. '''

    def __init__(self):

        self.stdscr = curses.initscr()         ''' Starts the curses standard screen. '''
        self.stdscr.addstr(0, 0, text.title)   ''' Prints the title. '''
        self.stdscr.refresh()                  ''' Refreshes the screen. '''

        ''' Creates the win_modes window for showing the available commands. '''
        self.win_modes = curses.newwin(14, 36, 9, 0)
        self.win_modes.addstr(0, 0, text.modalities)
        self.win_modes.refresh()

        ''' Creates the win_info window for printing information. '''
        self.win_info = curses.newwin(5, 45, 19, 39)
        self.win_info.addstr(0, 0, text.info)
        self.win_info.refresh()

        self.win_input = curses.newwin(1, 5, 23, 0)      ''' Window for acquiring the user input. '''
        self.win_request = curses.newwin(1, 35, 22, 0)   ''' Window for printing an input request. '''

    def clear_modes(self):
        ''' Clears and resets the win_modes. '''
        self.win_modes.clear()
        self.win_modes.addstr(0, 0, text.modalities)
        self.win_modes.refresh()

    def clear_info(self):
        ''' Clears and reset the win_info. '''
        self.win_info.clear()
        self.win_info.addstr(0, 0, text.info)
        self.win_info.refresh()

    def clear_input(self):
        ''' Clears the win_input. '''
        self.win_input.clear()
        self.win_input.refresh()

    def clear_request(self):
        ''' Clears the win_request. ''' 
        self.win_request.clear()
        self.win_request.refresh()

    def set_wasd(self):
        ''' Shows into the win_modes the command for directly drive the robot. '''
        self.win_modes.clear()
        self.win_modes.addstr(0, 0, text.wasd)
        self.win_modes.refresh()

    def command_not_valid(self):
        ''' Sends the command not valid message into win_request. '''
        self.win_request.clear()
        self.win_request.addstr(0, 0, "Command NOT valid")
        self.win_request.refresh()
