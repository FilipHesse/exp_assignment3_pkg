#!/usr/bin/env python3
"""Simulated User Interface

This script creates a ROS node, which simulates a user interface to
communicate with the robot pet.

It calls the pet_command service to send a command which consists of a
string to specify the command type and a robot_pet/Point2d to specify the
desired position in case it is a "go_to" command.

The programmer has chosen a service over a publisher, because we want to
make sure no message gets lost.

    Requirements:
        You can use the launchfile params.launch to set these to some 
        default values
"""

from __future__ import print_function
import rospy
from exp_assignment3_pkg.srv import *
import random

def pet_command_client():
    """Service client, that creates commands to simulate the users behavior 

    This function creates and sends two types of commands:
    1) "play" 0 0 to notify the robot to go to playing mode
    2) "go_to" x y to give the robot a target position
    
    Each fifth command is a play command, the other commands are go_to commands.
    Between two commands, there is always a rondom time passing between 0.5 and 5 seconds
    """

    rospy.logdebug("Wait for service pet_command to be available")
    rospy.wait_for_service('pet_command')

    map_width = rospy.get_param("/map_width")
    map_height = rospy.get_param("/map_height")

    counter = 0
    while not rospy.is_shutdown():
        try:
            pet_command = rospy.ServiceProxy('pet_command', PetCommand)

            #Fill the request
            request = PetCommandRequest()
            request.header = rospy.Header()

            #Each fivth command is play, the other commands are go_to a random
            #place within the range
            if (counter % 5) == 2:  # 2 because we do not want the very first command to be a play command
                request.command = "play"
                request.point.x = 0 #Values for x and y don't matter
                request.point.y = 0 
            else:
                request.command = "go_to"
                request.point.x = random.randint(0,map_width-1)
                request.point.y = random.randint(0,map_height-1) 
            
            rospy.loginfo("User sending command: {} x={} y={}".format(request.command, request.point.x, request.point.y))
            res = pet_command(request)
            
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)

        #incement counter
        counter += 1

        #Wait for a random time between 0.5 and 5 seconds 
        rospy.sleep(random.uniform(0.5, 5))


if __name__ == "__main__":
    """Entry point of script
    """
    rospy.init_node('ui')
    pet_command_client()