#!/usr/bin/env python3
"""Heart of robot_pet package: defines robots behavior

Contains a finite state machine implemented in smach. The 3 states of the
robot pet are NORMAL, PLAY and SLEEP. The state diagram can be found in the
README.md of the project

Each interface with the ROS infrastructure, such as service clients,
servers, action clients and publishers are implemented within separate
classes. All these interfaces are then passed to the smach-states while they
are constructed, in order to make the interfaces accessible for the states.

    Requirements:
        The following parameters need to be set in the ros parameter server:
            /map_width
            /map_height
        You can use the launchfile params.launch to set these to some 
        default values
"""

from __future__ import print_function
from exp_assignment3_pkg.srv import PetCommand, PetCommandResponse, PetCommandRequest, GetPosition, GetPositionRequest
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseGoal
from std_msgs.msg import Float64, Header
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped, Pose

import rospy
import actionlib
import smach
import random
import smach_ros
import states
import tf
import cfg

class PetCommandServer:
    """Server to process Pet Commands

    If a new command comes in, this class saves it as a member and sets a flag,
    that a new command is available.

    The member _command should not be accessed directly, but through the
    function get_new_command(), because that function automatically sets the
    _new_command_available-flag to False.


    Attributes:
        _command (bool): Content of last command
        _new_command_available (PetCommandRequest): Is a new unprocessed command available?
    """

    def __init__(self):
        """Initializes a service and the attributes
        """

        rospy.Service('pet_command', PetCommand, self.handle_command)
        rospy.loginfo("Ready to receive commands.")
        self._new_command_available = False 
        self._command = PetCommandRequest()    #Make variables private, so that get_new_command() has to be used


    def handle_command(self, req):
        """Saves command to Atrributes and returns an empty response to caller

        Args:
            req (PentCommandRequest): Service request containing a command and 
                optianally a targetpoint

        Returns:
            [PetCommandResponse]: Empty response
        """

        rospy.loginfo("Command received: {} {} {}".format(req.command, req.point.x, req.point.y))
        self._command = req
        self._new_command_available = True
        return PetCommandResponse()


    def is_new_command_available(self):
        """Call this function to check if new command is available

        Returns:
            bool: True if unprocessed command is available
        """

        return self._new_command_available


    def get_new_command(self):
        """Interface for state machine to get new command

        Returns:
            PetCommandRequest: Command as it was received by the server
        """

        self._new_command_available = False
        return self._command





class SetTargetActionClient():
    """Action client to set target position

    An action client has been chosen, because it is a non blocking call. This
    way, incoming commands can still be handeled properly and actions like
    "sleep" will be processed right after the service call has finished.

    To use this class, only use the functions call_action() to set a new target
    and check the value ready_for_new_target to check if previous action was 
    finished

    Attributes: 
        ready_for_new_target (bool): True if the last action 
            has been finished
        client (actionlib.SimpleActionClient): Clientobject to interface with 
            actual action 
    """
    def __init__(self):
        """Creates the client and waits for action server to be available
        """
        self.ready_for_new_target = True
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for action server to come up...")
        self.client.wait_for_server()

    def call_action(self, room):
        """Use this function to set a new target position of the robot_pet  

        Args:
            x (int): target x-position of the robot
            y (int): target y-position of the robot
        """
        rotationz=0

        self.ready_for_new_target = False

        ag = MoveBaseGoal()
        ag.target_pose.header.frame_id = "map"
        ag.target_pose.pose.position.x = float(x)
        ag.target_pose.pose.position.y = float(y)
        ag.target_pose.pose.position.z = float(0)
        dummy = tf.transformations.quaternion_from_euler(0, 0, rotationz)
        ag.target_pose.pose.orientation.x = dummy[0]
        ag.target_pose.pose.orientation.y = dummy[1]
        ag.target_pose.pose.orientation.z = dummy[2]
        ag.target_pose.pose.orientation.w = dummy[3]


        self.client.send_goal(ag,
                        done_cb=self.callback_done)

        rospy.loginfo("Goal (x={}, y={}) has been sent to the action server.".format(x, y))

    def callback_done(self, state, result):
        """This callback gets called when action server is done

        Sets attribute ready_for_new_target to true

        Args:
            state (state of action): Status of the action according to
                http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
            result (SetTargetPositionResult): Result of action: Position of the point
                that was reached
        """
        rospy.loginfo("SetTargetAction is done, position x={} y={} reached. Action state: {}".format(result.final_position.x, result.final_position.y, state))
        self.ready_for_new_target = True



class SleepingTimer():
    """Timer Class, that schedules the sleeping times

    Contains a timer running in one-shot-mode. This allows to define different
    times for sleeping and being awake each time the timer has elapsed

    Usage: Check the flag time_to_sleep to check if robot should be sleeping
    right now or if it should be awake

    Attributes:
        sleeping_time_range ((int, int)): Sleeping time will be random between 10
            and 15 seconds
        awake_time_range  ((int, int)): Awake time will be random between 20 and 30 seconds
        time_to_sleep (bool): Flag for the user of this class to check if its time to sleep
            (True) or time to be awake (False)
        timer (rospy.Timer): Timer that triggers the callbacks
    """
    def __init__(self):
        """Initialize attributes
        """
        self.sleeping_time_range = (10, 15)  #Sleep between 10 and 15 seconds
        self.awake_time_range = (20, 30)  #Be awake for ...
        self.time_to_sleep = False
        self.timer = rospy.Timer(rospy.Duration(random.uniform(*self.awake_time_range)), self.callback, oneshot=True)

    def callback(self, msg):
        """Get called when self.timer has elapsed

        Toggles the flag time_to_sleep and restarts the timer with appropriate
        random time

        Args:
            msg (??): unused
        """
        self.time_to_sleep = not self.time_to_sleep

        if self.time_to_sleep:
            rospy.loginfo("It's time to go to bed!")
            self.timer = rospy.Timer(rospy.Duration(random.uniform(*self.sleeping_time_range)), self.callback, oneshot=True)
        else:
            rospy.loginfo("It's time to wake up!")
            self.timer = rospy.Timer(rospy.Duration(random.uniform(*self.awake_time_range)), self.callback, oneshot=True)



if __name__ == "__main__":
    """Main function of this script

    Instanciates all classes, that have been defined above. Creates State
    machine with all the states and spins for callbacks
    """
    rospy.init_node('behavior_state_machine')




    pet_command_server = PetCommandServer()
    set_target_action_client = SetTargetActionClient()
    sleeping_timer = SleepingTimer()

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=[])

    # Open the container
    with sm_top:
        # Add states to the container
        smach.StateMachine.add('NORMAL', states.Normal(pet_command_server, set_target_action_client, sleeping_timer), 
                               transitions={'cmd_play':'PLAY', 
                                            'sleeping_time':'SLEEP'})

        smach.StateMachine.add('SLEEP', states.Sleep(pet_command_server, set_target_action_client, sleeping_timer), 
                               transitions={'slept_enough':'NORMAL'})

        smach.StateMachine.add('PLAY', states.Play(pet_command_server, set_target_action_client, sleeping_timer), 
                               transitions={'played_enough':'NORMAL',
                                            'sleeping_time':'SLEEP' })


    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start() 

    # Execute SMACH plan
    outcome = sm_top.execute()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass