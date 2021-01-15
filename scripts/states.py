import smach
import random
import smach_ros
import rospy
import cfg
import numpy as np
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate


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

    def __init__(self, pet_command_server, set_target_action_client, sleeping_timer):
        """Initializes attributes and reads ros parameters (width, height)

        Args:
            get_position_client (GetPositionClient): See class description
            pet_command_server (PetCommandServer): See class description
            set_target_action_client (SetTargetActionClient): See class description
            sleeping_timer (SleepingTimer): See class description
        """

        smach.State.__init__(self, outcomes=['cmd_play','sleeping_time'])
        
        self.pet_command_server = pet_command_server
        self.set_target_action_client = set_target_action_client
        self.sleeping_timer = sleeping_timer
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
                    #First reach next position then go to sleeping state
                    while not self.set_target_action_client.ready_for_new_target:
                        rate.sleep()
                    return 'cmd_play'
                if cmd.command =='go_to':
                    rospy.loginfo("Invalid command 'go_to' for state NORMAL. First say 'play' and then give go_to commands!")

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
        smach.State.__init__(self, outcomes=['played_enough','sleeping_time'])
        self.pet_command_server = pet_command_server
        self.set_target_action_client = set_target_action_client
        self.sleeping_timer = sleeping_timer
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
        played (random between 1 and 3) has been reached.

        Args:
            userdata (---): unused

        Returns:
            string: Outcomes of this state: "played_enough" or "sleeping_time"
        """
        rospy.loginfo('----------------------------------------------\n------------------------------ ENTERING STATE PLAY ---\n--------------------------------------------------------------------------')
        rate = rospy.Rate(10)

        games_to_play = random.randint(1,3)

        number_games = 0
        while True:
            number_games += 1
            #Get Persons Position
            # x,y = get_position_client.call_srv("user")
            #Go To Person
            set_target_action_client.call_action(x,y)
            #Wait until position reached
            while not self.set_target_action_client.ready_for_new_target:
                rate.sleep()
            
            #Discard all previous commands
            if self.pet_command_server.is_new_command_available():
                # Ignore user commands -> Get the commands to consider command as hadeled
                cmd = self.pet_command_server.get_new_command()
                rospy.loginfo("Previous commands were ignored, because robot was not ready to receive commands")
            
            valid_target = False
            #Check commands until a valid one comes in (go to)
            while not valid_target:
                #Wait for command
                while not self.pet_command_server.is_new_command_available():
                    rate.sleep()

                cmd = self.pet_command_server.get_new_command()
                if cmd.command == 'play':
                    rospy.loginfo("Robot is already playing")

                if cmd.command =='go_to':
                    valid_target = True
                    room = cmd.room


            # Send pointer position to map
            # pointer_pos = Point2dOnOff()
            # pointer_pos.point.x = x
            # pointer_pos.point.y = y
            # pointer_pos.on =  True  #SWITCH ON
            # self.pub.publish(pointer_pos)

            #Go To Target
            set_target_action_client.call_action(x,y)
            
            #Wait until position reached
            while not self.set_target_action_client.ready_for_new_target:
                rate.sleep()

            # Send pointer position to map
            #pointer_pos.on =  False     #SWITCH OFF
            #self.pub.publish(pointer_pos)

            #Get Persons Position
            # x,y = get_position_client.call_srv("user")
            
            #Go To Person
            set_target_action_client.call_action(x,y)
            
            #Wait until position reached
            while not self.set_target_action_client.ready_for_new_target:
                rate.sleep()
            #Check if time to sleep
            if self.sleeping_timer.time_to_sleep:
                rospy.loginfo("I am tired. Good night!")
                return 'sleeping_time'

            #Check if played enough
            if games_to_play == number_games:
                rospy.loginfo("I played {} games. This is enough".format(number_games))
                return 'played_enough'

            rate.sleep()