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
from actionlib_msgs.msg import GoalID, GoalStatus
from geometry_msgs.msg import PoseStamped, Pose
from exp_assignment3_pkg.msg import EmptyAction, EmptyGoal, BallVisible, WhatIsGoingOn
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker

import rospy
import actionlib
import smach
import random
import smach_ros
import states
import tf
import room_info

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
        self._command_processing = PetCommandRequest()


    def handle_command(self, req):
        """Saves command to Atrributes and returns an empty response to caller

        Args:
            req (PentCommandRequest): Service request containing a command and 
                optianally a targetpoint

        Returns:
            [PetCommandResponse]: Empty response
        """

        rospy.loginfo("Command received: {} {}".format(req.command, req.room))
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

    def process_command(self):
        self._command_processing = self._command

    def processing_command_done(self):
        self._command_processing = PetCommandRequest()  #Empty



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

    def call_action(self, x, y):
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
        rospy.loginfo("MoveBaseAction is done! State: {}".format(state))
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
        self.sleeping_time_range = (rospy.get_param("/sleeping_time_min"), rospy.get_param("/sleeping_time_max")) 
        self.awake_time_range = (rospy.get_param("/awake_time_min"), rospy.get_param("/awake_time_max")) 
        self.time_to_sleep = False
        if (rospy.get_param("/run_sleeping_timer")):
            duration = rospy.Duration(random.uniform(*self.awake_time_range))
        else:   #If robot should not sleep: stay awake for a veeeeeerry long time
            duration = rospy.Duration(1000000000)
        self.next_callback = rospy.get_rostime() + duration
        self.timer = rospy.Timer(duration, self.callback, oneshot=True)

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
            duration = rospy.Duration(random.uniform(*self.sleeping_time_range))
            self.timer = rospy.Timer(duration, self.callback, oneshot=True)
        else:
            rospy.loginfo("It's time to wake up!")
            duration = rospy.Duration(random.uniform(*self.awake_time_range))
            self.timer = rospy.Timer(duration, self.callback, oneshot=True)
        self.next_callback = rospy.get_rostime() + duration

    def seconds_till_sleep_or_wakeup(self):
        return ( self.next_callback - rospy.get_rostime()).secs 

class BallVisibleSubscriber:
    """Subscriber, that subscribes to the topic camera1/ball_visible
    """

    def __init__(self):
        """Creates the subscriber
        """
        self.ball_visible = False
        self.sub = rospy.Subscriber(
            "camera1/ball_visible", BallVisible, self.callback)

    def callback(self, msg):
        """Publisher callback

        Args:
            msg (Bool): is ball visible
        """
        self.ball_visible = msg.visible.data
        self.color = msg.color.data

    def is_ball_visible(self):
        """Use this function to check if ball was visible in the last message

        Returns:
            bool: Is ball visible?
        """
        return self.ball_visible


class FollowBallActionClient():
    """Action client to make the robot follow the ball

    An action client has been chosen, because it is a non blocking call.

    Attributes: 
        client (actionlib.SimpleActionClient): Clientobject to interface with 
            actual action 
    """

    def __init__(self):
        """Creates the client and waits for action server to be available
        """
        self.client = actionlib.SimpleActionClient(
            'follow_ball', EmptyAction)
        rospy.loginfo(
            "follow_ball: Waiting for action server to come up...")
        self.client.wait_for_server()

    def call_action(self):
        """Use this function to make the robot follow the ball  
        """

        rospy.loginfo(
            "follow_ball: Action server has been called")

        if self.is_active():
            rospy.loginfo(
                "follow_ball: Trying to follow ball, but action server already busy doing it")
            return

        goal = EmptyGoal()
        self.client.send_goal(goal,
                              done_cb=self.callback_done)

    def callback_done(self, state, result):
        """This callback gets called when action server is done

        Args:
            state (state of action): Status of the action according to
                http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
            result (SetTargetPositionResult): Result of action: Position of the point
                that was reached
        """
        rospy.loginfo(
            "FollowBallAction is done. Action state: {}".format(state))

    def cancel_goal(self):
        """Cancel current goal of action server
        """
        self.client.cancel_goal()

    def is_active(self):
        """Is action server currently processing a goal?

        Returns:
            bool: is action server currently processing a goal
        """
        return self.client.get_state() == GoalStatus.ACTIVE

    def done_successful(self):
        if self.client.get_state() == GoalStatus.SUCCEEDED:
            return True
        else:
            return False

class StatusMessagePublisher:
    def __init__(self, state_machine_top, state_machine_normal, state_machine_find, sleeping_timer, pet_command_server, state_play):
        self.status_message_timer = rospy.Timer(rospy.Duration(1), self.status_message_callback)
        self.pub_whatsup = rospy.Publisher("/what_is_going_on", WhatIsGoingOn, queue_size=10)
        self.pub_room = rospy.Publisher("/room_info", Marker, queue_size=10)
        self.state_machine_top = state_machine_top
        self.state_machine_normal = state_machine_normal
        self.state_machine_find = state_machine_find
        self.sleeping_timer = sleeping_timer
        self.pet_command_server = pet_command_server
        self.state_play = state_play

    def status_message_callback(self, timer):
        # Create message What Is going on
        msg = WhatIsGoingOn()
        msg.state___________________ = self.state_machine_top.get_active_states()[0]
        msg.substate_normal_________ = self.state_machine_normal.get_active_states()[0]
        msg.substate_find___________ = self.state_machine_find.get_active_states()[0]
        msg.sleep_timer_info________ = f"Now its time to sleep. Wake up in {sleeping_timer.seconds_till_sleep_or_wakeup()} s" if sleeping_timer.time_to_sleep else f"Now its time to be awake. Sleeping time in {sleeping_timer.seconds_till_sleep_or_wakeup()} s"
        msg.last_command____________ = f"Command: {self.pet_command_server._command.command}, room: {self.pet_command_server._command.room}"
        msg.command_processing______ = f"Command: {self.pet_command_server._command_processing.command}, room: {self.pet_command_server._command_processing.room}"
        msg.PLAY_game_info__________ = f"Played {state_play.number_games} out of {state_play.games_to_play} games"
        msg.PLAY_waiting_for_command = state_play.waiting_for_target_command

        msg.room_info_0 = f"Color: {room_info.info[0].color}, name: {room_info.info[0].name}, x: {room_info.info[0].x}, y: {room_info.info[0].y}"
        msg.room_info_1 = f"Color: {room_info.info[1].color}, name: {room_info.info[1].name}, x: {room_info.info[1].x}, y: {room_info.info[1].y}"
        msg.room_info_2 = f"Color: {room_info.info[2].color}, name: {room_info.info[2].name}, x: {room_info.info[2].x}, y: {room_info.info[2].y}"
        msg.room_info_3 = f"Color: {room_info.info[3].color}, name: {room_info.info[3].name}, x: {room_info.info[3].x}, y: {room_info.info[3].y}"
        msg.room_info_4 = f"Color: {room_info.info[4].color}, name: {room_info.info[4].name}, x: {room_info.info[4].x}, y: {room_info.info[4].y}"
        msg.room_info_5 = f"Color: {room_info.info[5].color}, name: {room_info.info[5].name}, x: {room_info.info[5].x}, y: {room_info.info[5].y}"

        self.pub_whatsup.publish(msg)

        #Create message room_info (Markers for rviz)
        for i, room in enumerate(room_info.info):
            if room.positions_known():
                #Ball
                marker = Marker()
                marker.header.frame_id = "map"
                marker.ns = "balls"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = room.x
                marker.pose.position.y = room.y
                marker.pose.position.z = 0.5
                marker.pose.orientation.x = 0.
                marker.pose.orientation.y = 0.
                marker.pose.orientation.z = 0.
                marker.pose.orientation.z = 1.

                marker.color.r = room.r
                marker.color.g = room.g
                marker.color.b = room.b
                marker.color.a = 1.0

                
                marker.scale.x = 1
                marker.scale.y = 1
                marker.scale.z = 1

                marker.lifetime = rospy.Duration()

                self.pub_room.publish(marker)

                #Text
                marker = Marker()
                marker.header.frame_id = "map"
                marker.ns = "captions"
                marker.id = i+10
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD
                marker.pose.position.x = room.x
                marker.pose.position.y = room.y
                marker.pose.position.z = 1.5
                marker.pose.orientation.x = 0.
                marker.pose.orientation.y = 0.
                marker.pose.orientation.z = 0.
                marker.pose.orientation.z = 1.

                marker.color.r = 0.
                marker.color.g = 0.
                marker.color.b = 0.
                marker.color.a = 1.0

                
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0

                marker.text = room.name

                marker.lifetime = rospy.Duration()

                self.pub_room.publish(marker)

if __name__ == "__main__":
    """Main function of this script

    Instanciates all classes, that have been defined above. Creates State
    machine with all the states and spins for callbacks
    """
    rospy.init_node('behavior_state_machine')

    pet_command_server = PetCommandServer()
    set_target_action_client = SetTargetActionClient()
    sleeping_timer = SleepingTimer()
    follow_ball_action_client = FollowBallActionClient()
    ball_visible_subscriber = BallVisibleSubscriber()

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=[])

    # Open the container
    with sm_top:

        sm_normal = smach.StateMachine(outcomes=['normal_cmd_play','normal_sleeping_time'],
                                       output_keys=['init_games'])
        # Add states to the container
        with sm_normal:

            # Add states to the container 
            smach.StateMachine.add('NORMAL_DEFAULT', states.Normal(pet_command_server, set_target_action_client, sleeping_timer, ball_visible_subscriber), 
                               transitions={'cmd_play':'normal_cmd_play', 
                                            'sleeping_time':'normal_sleeping_time',
                                            'track':'NORMAL_TRACK'})
            smach.StateMachine.add('NORMAL_TRACK', states.Track(ball_visible_subscriber, follow_ball_action_client),
                                   transitions={'tracking_done':'NORMAL_DEFAULT'})

        smach.StateMachine.add('NORMAL', sm_normal,
                               transitions={'normal_cmd_play':'PLAY', 
                                            'normal_sleeping_time':'SLEEP'})

        smach.StateMachine.add('SLEEP', states.Sleep(pet_command_server, set_target_action_client, sleeping_timer), 
                               transitions={'slept_enough':'NORMAL'})

        state_play = states.Play(pet_command_server, set_target_action_client, sleeping_timer)
        smach.StateMachine.add('PLAY', state_play, 
                               transitions={'played_enough':'NORMAL',
                                            'sleeping_time':'SLEEP',
                                            'find':'FIND' })

        sm_find = smach.StateMachine(outcomes=['find_target_location_found','find_sleeping_time'],
                                     input_keys=['find_color'],
                                     output_keys=['init_games']) 
        # Add states to the container
        with sm_find:

            # Add states to the container 
            smach.StateMachine.add('FIND_DEFAULT', states.Find(ball_visible_subscriber, sleeping_timer), 
                               transitions={'target_location_found':'find_target_location_found', 
                                            'sleeping_time':'find_sleeping_time',
                                            'track':'FIND_TRACK'})
            smach.StateMachine.add('FIND_TRACK', states.Track(ball_visible_subscriber, follow_ball_action_client),
                                   transitions={'tracking_done':'FIND_DEFAULT'},
                                   remapping={'ball_color':'track_color'})

        smach.StateMachine.add('FIND', sm_find,
                               transitions={'find_target_location_found':'PLAY', 
                                            'find_sleeping_time':'SLEEP'})
                            

    status_message_publisher = StatusMessagePublisher(sm_top, sm_normal, sm_find, sleeping_timer, pet_command_server, state_play)

    # Execute SMACH plan
    outcome = sm_top.execute()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass