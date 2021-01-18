import smach
import random
import smach_ros
import rospy
import room_info
import numpy as np
import tf
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from std_srvs.srv import Empty, EmptyRequest
from actionlib_msgs.msg import GoalID, GoalStatus


########################################################
## STATE MACHINE CODE
#######################################################

class Normal(smach.State):
    """Defines the Smach-state NORMAL

    In this state the robot goes from one random target to another

    Attributes:
        get_position_client (GetPositionClient): Service client to get position of an object
        pet_command_server (PetCommandServer): Allows accessing the last command from the user
        set_target_action_client (SetTargetActionClient): Action client to set a new target position
        sleeping_timer (SleepingTimer): Allows checking if it is time to sleep
        map_width (int): Width of map to choose appropriate target positions
        map_height (int): Height of map to choose appropriate target positions
    """

    def __init__(self, pet_command_server, set_target_action_client, sleeping_timer, ball_visible_subscriber):
        """Initializes attributes and reads ros parameters (width, height)

        Args:
            get_position_client (GetPositionClient): See class description
            pet_command_server (PetCommandServer): See class description
            set_target_action_client (SetTargetActionClient): See class description
            sleeping_timer (SleepingTimer): See class description
        """

        smach.State.__init__(self, outcomes=['cmd_play','sleeping_time','track'],
                                   output_keys=['ball_color','init_games'])
        
        self.pet_command_server = pet_command_server
        self.set_target_action_client = set_target_action_client
        self.sleeping_timer = sleeping_timer
        self.ball_visible_subscriber = ball_visible_subscriber
        self.costmap_subscriber = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.callback_costmap)
        self.costmap_update_subscriber = rospy.Subscriber("/move_base/global_costmap/costmap_updates", OccupancyGridUpdate, self.callback_costmap_update)

        self.costmap = np.array([])

    def callback_costmap(self, msg):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_orig_x = msg.info.origin.position.x
        self.map_orig_y = msg.info.origin.position.y
        self.costmap = np.array(msg.data).reshape(self.map_height, self.map_width)

    def callback_costmap_update(self, msg):
        self.map_width = msg.width
        self.map_height = msg.height
        self.costmap = np.array(msg.data).reshape(self.map_height, self.map_width)

    def execute(self, userdata):
        """ Robot moves around between random positions

        Endless loop checks if it's time to sleep or if a user command has been
        sent to exit this state. Then it sends new position targets in case the
        robot is not already moving

        Args: userdata (----): unused

        Returns: string: outcomes "cmd_play" or "sleeping time
        """

        rospy.loginfo('----------------------------------------------\n------------------------------ ENTERING STATE NORMAL ---\n--------------------------------------------------------------------------')
        rate = rospy.Rate(10)
        while True:
            #Check if its time to sleep
            if self.sleeping_timer.time_to_sleep:
                self.set_target_action_client.client.cancel_all_goals()
                return 'sleeping_time'

            # React to user commands
            if self.pet_command_server.is_new_command_available():
                cmd = self.pet_command_server.get_new_command()
                if cmd.command == 'play' :
                    self.pet_command_server.process_command()
                    # Abort all current goals
                    self.set_target_action_client.client.cancel_all_goals()
                    userdata.init_games = True
                    return 'cmd_play'
                if cmd.command =='go_to':
                    rospy.loginfo("Invalid command 'go_to' for state NORMAL. First say 'play' and then give go_to commands!")

            # Track ball if new ball visible
            if self.ball_visible_subscriber.is_ball_visible() and not room_info.is_color_known(self.ball_visible_subscriber.color):
                rospy.loginfo(f"Ball of unknown color ({self.ball_visible_subscriber.color})detected. Switch to state tracking")
                self.set_target_action_client.client.cancel_all_goals()
                userdata.ball_color = self.ball_visible_subscriber.color
                return 'track'

            # Normal behavior: set random targets
            if self.set_target_action_client.ready_for_new_target:
                i = 1
                while self.costmap.size == 0:
                    if i == 1:
                        rospy.loginfo("Waiting for costmap in order to generate a target position.")
                        rate.sleep()

                max_x = self.map_orig_x + self.map_width * self.map_resolution
                max_y = self.map_orig_y + self.map_height * self.map_resolution
                # Implement Do.. while loop
                condition = True
                while condition:
                    next_x = random.uniform(self.map_orig_x, max_x)
                    next_y = random.uniform(self.map_orig_y, max_y)
                    condition = not self.is_costmap_free(next_x, next_y)

                self.set_target_action_client.call_action(next_x, next_y)
            
            rate.sleep()

    def is_costmap_free(self, x, y):
        x_idx = int((x - self.map_orig_x)/self.map_resolution)
        y_idx = int((y - self.map_orig_y)/self.map_resolution)

        if self.costmap[y_idx, x_idx] == 0:
            return True
        else:
            return False



class Sleep(smach.State):
    """Defines the Smach-state SLEEP

    In this state the robot goes to the house and stay there until sleeping time is over

    Attributes:
        get_position_client (GetPositionClient): Service client to get position of an object
        pet_command_server (PetCommandServer): Allows accessing the last command from the user
        set_target_action_client (SetTargetActionClient): Action client to set a new target position
        sleeping_timer (SleepingTimer): Allows checking if it is time to sleep
    """
    def __init__(self, pet_command_server, set_target_action_client, sleeping_timer):
        """Initializes attributes and reads ros parameters (width, height)

        Args:
            get_position_client (GetPositionClient): See class description
            pet_command_server (PetCommandServer): See class description
            set_target_action_client (SetTargetActionClient): See class description
            sleeping_timer (SleepingTimer): See class description
        """

        smach.State.__init__(self, outcomes=['slept_enough'])

        self.pet_command_server = pet_command_server
        self.set_target_action_client = set_target_action_client
        self.sleeping_timer = sleeping_timer

    def execute(self, userdata):
        """Robot goes to house and sleeps until sleeping time over

        Args:
            userdata (----): unused

        Returns:
            string: outcome: slept_enough
        """
        rospy.loginfo('----------------------------------------------\n------------------------------ ENTERING STATE SLEEP ---\n--------------------------------------------------------------------------')
        rate = rospy.Rate(10)
        
        # # Robot might still be moving => wait until last target reached
        # while not self.set_target_action_client.ready_for_new_target:
        #     rate.sleep()
        
        #Get position of house
        # x,y = get_position_client.call_srv("house")
        
        #Set new target: house
        self.set_target_action_client.call_action(rospy.get_param("/house_x"),rospy.get_param("/house_y"))

        #just wait until wake-up flag is set
        while True:
            
            if self.pet_command_server.is_new_command_available():
                # Ignore user commands -> Get the commands to consider command as hadeled
                cmd = self.pet_command_server.get_new_command()
                rospy.loginfo("Command is ignored, because Robot is sleeping")

            if not self.sleeping_timer.time_to_sleep:
                return 'slept_enough'
            rate.sleep()
            

# define state Play
class Play(smach.State):
    """Defines the Smach-state PLAY

    In this state the robot performs the following actions in a loop:
    1) Go to user
    2) Wait for a command that specifies a new target
    3) Go to new target
    Repeat

    The game is repeated for a randum number of times between 1 and 3

    Attributes:
        get_position_client (GetPositionClient): Service client to get position of an object
        pet_command_server (PetCommandServer): Allows accessing the last command from the user
        set_target_action_client (SetTargetActionClient): Action client to set a new target position
        sleeping_timer (SleepingTimer): Allows checking if it is time to sleep
        pub (rospy.Publisher()): A publisher to publish the pointer positions. This publisher is
            defined inside this state because it is not needed in any other state
    """
    def __init__(self, pet_command_server, set_target_action_client, sleeping_timer):
        """Initializes attributes and reads ros parameters (width, height)

        Args:
            get_position_client (GetPositionClient): See class description
            pet_command_server (PetCommandServer): See class description
            set_target_action_client (SetTargetActionClient): See class description
            sleeping_timer (SleepingTimer): See class description
        """
        smach.State.__init__(self, outcomes=['played_enough','sleeping_time', 'find'],
                                   output_keys=['find_color'],
                                   input_keys=['init_games'])
        self.pet_command_server = pet_command_server
        self.set_target_action_client = set_target_action_client
        self.sleeping_timer = sleeping_timer
        self.number_games = 0
        self.games_to_play = 0
        self.waiting_for_target_command = False
        #self.pub = rospy.Publisher('pointer_position', Point2dOnOff, queue_size=10) #define this publisher only here, because pointer position is not needed anywhere else

    def execute(self, userdata):
        """Executes play mode

        In this state the robot performs the following actions in a loop:
        1) Go to user
        2) Wait for a command that specifies a new target
        3) Go to new target 
        4) Go back to person
        
        This function implements these steps and braks them down into smaller
        substeps Details can be viewed inside the code, which is commented. It
        is only checked at the end of each game (after going back to the
        person), if it is time to go to sleep or if the number of games to be
        played (random between 2 and 4) has been reached.

        Args:
            userdata (---): unused

        Returns:
            string: Outcomes of this state: "played_enough" or "sleeping_time"
        """
        rospy.loginfo('----------------------------------------------\n------------------------------ ENTERING STATE PLAY ---\n--------------------------------------------------------------------------')
        rate = rospy.Rate(10)

        if userdata.init_games: # Coming from state normal
            self.games_to_play = random.randint(2,4)
            self.number_games = 0
        else:   #Coming from state find
            self.number_games +=1
            if self.games_to_play == self.number_games:
                rospy.loginfo("I played {} games. This is enough".format(self.number_games))
                return 'played_enough'

        while True:
            #Get Persons Position
            # x,y = get_position_client.call_srv("user")
            #Go To Person
            
            self.set_target_action_client.call_action(rospy.get_param("/user_x"),rospy.get_param("/user_y"))
            #Wait until position reached
            while not self.set_target_action_client.ready_for_new_target:
                rate.sleep()
            
            #Discard all previous commands
            if self.pet_command_server.is_new_command_available():
                # Ignore user commands -> Get the commands to consider command as hadeled
                cmd = self.pet_command_server.get_new_command()
                rospy.loginfo("Previous commands were ignored, because robot was not ready to receive commands")
            
            valid_target = False
            self.pet_command_server.processing_command_done()
            #Check commands until a valid one comes in (go to)
            while not valid_target:
                #Wait for command
                self.waiting_for_target_command = True
                while not self.pet_command_server.is_new_command_available():
                    rate.sleep()

                cmd = self.pet_command_server.get_new_command()
                if cmd.command == 'play':
                    rospy.loginfo("Robot is already playing")

                if cmd.command =='go_to':
                    self.pet_command_server.process_command()
                    valid_target = True
                    room_name = cmd.room

                #Check if time to sleep
                if self.sleeping_timer.time_to_sleep:
                    rospy.loginfo("I am tired. Good night!")
                    self.pet_command_server.processing_command_done()
                    self.waiting_for_target_command = False
                    return 'sleeping_time'

            self.waiting_for_target_command = False
            target_room_info = room_info.get_room_info_by_name(room_name)

            if target_room_info.positions_known():
                #Go To Target
                self.set_target_action_client.call_action(target_room_info.x,target_room_info.y)
            else:
                userdata.find_color=target_room_info.color
                return 'find'
            
            #Wait until position reached
            while not self.set_target_action_client.ready_for_new_target:
                #Check if time to sleep
                if self.sleeping_timer.time_to_sleep:
                    rospy.loginfo("I am tired. Good night!")
                    self.pet_command_server.processing_command_done()
                    self.waiting_for_target_command = False
                    return 'sleeping_time'
                rate.sleep()

            
            #Remain 2 seconds in target position
            rospy.sleep(2.)
            
            #Go To Person
            self.set_target_action_client.call_action(rospy.get_param("/user_x"),rospy.get_param("/user_y"))

            #Wait until position reached
            while not self.set_target_action_client.ready_for_new_target:
                #Check if time to sleep
                if self.sleeping_timer.time_to_sleep:
                    rospy.loginfo("I am tired. Good night!")
                    self.pet_command_server.processing_command_done()
                    self.waiting_for_target_command = False
                    return 'sleeping_time'
                rate.sleep()


            #Check if played enough
            self.number_games += 1
            if self.games_to_play == self.number_games:
                rospy.loginfo("I played {} games. This is enough".format(self.number_games))
                return 'played_enough'

            rate.sleep()

class Track(smach.State):
    def __init__(self, ball_visible_subscriber, follow_ball_action_client):
        self.ball_visible_subscriber = ball_visible_subscriber
        self.follow_ball_action_client = follow_ball_action_client
        smach.State.__init__(self, outcomes=['tracking_done'],
                                   input_keys=['ball_color'])

        self.listener = tf.TransformListener()

        hz = 10
        self.rate = rospy.Rate(hz)
        no_ball_seconds = 1
        self.iterations_no_ball = no_ball_seconds * hz

    def execute(self, userdata):
        rospy.loginfo('----------------------------------------------\n------------------------------ ENTERING STATE TRACK ---\n--------------------------------------------------------------------------')

        counter_no_ball = 0

        # Call action client!
        self.follow_ball_action_client.call_action()
        rospy.loginfo("Follow Ball action activated")
        while True:
            # If can not see ball for 3 seconds: abort running action and go back to normal state
            if not self.ball_visible_subscriber.is_ball_visible():
                counter_no_ball += 1
            else:
                counter_no_ball = 0

            # ball moved away again
            if counter_no_ball >= self.iterations_no_ball:
                rospy.loginfo("lost ball while tracking => abort tracking")
                self.cancel_goal_and_wait_till_done()
                return 'tracking_done'

            # Seeing different color => leave tracking
            if not self.ball_visible_subscriber.color == userdata.ball_color:
                rospy.loginfo("Seeing ball of different color")
                self.cancel_goal_and_wait_till_done()
                return 'tracking_done'

            if self.follow_ball_action_client.done_successful():
                target_room_info = room_info.get_room_info_by_color(userdata.ball_color)
                rospy.loginfo(f'{target_room_info.color} ball ({target_room_info.name}) was reached!') 
                try:
                    (trans,rot) = self.listener.lookupTransform('/map', '/link_chassis', rospy.Time(0))
                    target_room_info.x = trans[0]
                    target_room_info.y = trans[1]
                    rospy.loginfo('Saving Position ({target_room_info.x},{target_room_info.x})')
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    target_room_info.x = None
                    target_room_info.y = None
                    rospy.loginfo('TF between map and link_chassis could not be retrieved, saving position not possible')
                
                return 'tracking_done'
            #Check if goal reached!! and save current position to room

            self.rate.sleep()

    def cancel_goal_and_wait_till_done(self):
        """Cancels goal and waits till server has really stopped

        Args:
            rate (rospy.Rate): rate for polling sleep
        """
        if self.follow_ball_action_client.is_active():
            self.follow_ball_action_client.cancel_goal()
            # Wait unitl client indeed is not active anymore (robot should have stopped)
            while self.follow_ball_action_client.is_active():
                self.rate.sleep()

class Find(smach.State):
    def __init__(self, ball_visible_subscriber, sleeping_timer):
        smach.State.__init__(self, outcomes=['target_location_found', 'sleeping_time', 'track'],
                                   input_keys=['find_color'],
                                   output_keys=['track_color','init_games'])

        self.ball_visible_subscriber = ball_visible_subscriber
        self.explore_start = rospy.ServiceProxy('/explore/start', Empty)
        self.explore_stop = rospy.ServiceProxy('/explore/stop', Empty)
        self.sleeping_timer = sleeping_timer

        hz = 10
        self.rate = rospy.Rate(hz)

    def execute(self, userdata):
        rospy.loginfo('----------------------------------------------\n------------------------------ ENTERING STATE FIND ---\n--------------------------------------------------------------------------')

        #Which color am I trying to find?
        #Was color given in input key? If not,sear for color in membervariable
        #(Explanation: When coming from track substate, I dont get a new color to search for, only from PLAY)
        try:
            color = userdata.find_color
            self.find_color = color
        except NameError:
            pass

    
        #Do I already know the location of this color?
        if room_info.is_color_known(self.find_color):
            rospy.loginfo(f'Position of the {self.find_color} ball in known now! Go back to play state')
            userdata.init_games = False
            return 'target_location_found' #Yes -> GoTo PLAY
        else:
            rospy.loginfo(f'Trying to find the {self.find_color} ball')
            req = EmptyRequest()
            self.explore_start.call(req)#No -> Activate explore

        #While True
        while( True ):
            #Seeing unknown ball?
            #-> Go to track

            #Check if time to sleep
            if self.sleeping_timer.time_to_sleep:
                rospy.loginfo("I am tired. Good night!")
                #stop exploring
                req = EmptyRequest()
                self.explore_stop.call(req)
                return 'sleeping_time'

            if self.ball_visible_subscriber.is_ball_visible() and not room_info.is_color_known(self.ball_visible_subscriber.color):
                rospy.loginfo(f"An unknown ball ({self.ball_visible_subscriber.color})was detected")
                #stop exploring
                req = EmptyRequest()
                self.explore_stop.call(req)
                #go to state track
                userdata.track_color = self.ball_visible_subscriber.color
                return 'track'

            self.rate.sleep()



