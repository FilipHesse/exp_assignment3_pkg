"""Python file containing all knowledge about a room in the scenario

This Python file does not have a main, it simply defines a variable
"info", that is a list of all rooms with their positions (if known)
and colors. It will only be included by one node: behavior_state_machine
All this code could actually be written inside the 
behavior_state_machine.py file, but for cleanliness it is written in
a separate file
"""
import rospy

class RoomInfo:
    """Helper class to wrap information about one room
    """
    def __init__(self, color, name, r, g, b, x=None, y=None):
        """constructor

        Args:
            color (string): color of the ball
            name (string): name of the room
            r (float): red value [0.0, 1.0] of the ball, that will be displayed in RVIZ as a marker.
            g (float): green value [0.0, 1.0] of the ball, that will be displayed in RVIZ as a marker.
            b (float): blue value [0.0, 1.0] of the ball, that will be displayed in RVIZ as a marker.
            x (float, optional): X-Position of the room. None if position Unknown. Defaults to None.
            y (float, optional): Y-Position of the room. Defaults to None.
        """
        self.color = color  
        self.name = name 
        self.x = x
        self.y = y
        self.r = r
        self.g = g
        self.b = b
    
    def positions_known(self):
        """Return, if the positions of this room object are known

        Returns:
            Bool: False, if x-position or y-position are == None, else return true
        """
        if self.x == None or self.y == None:
            return False
        else:
            return True

# Global variable with room information
info=[RoomInfo("blue", "entrance", 0., 0., 1.),
    RoomInfo("red", "closet", 1., 0., 0.),
    RoomInfo("green", "living_room", 0., 1., 0.),
    RoomInfo("yellow", "kitchen", 1., 1., 0.),
    RoomInfo("pink", "bathroom", 1., 0., 1.),
    RoomInfo("black", "bedroom", 0., 0., 0.)]


def get_room_info_by_name(name):
    """Returns room object, that has specified name

    Args:
        name (string): name of the room

    Returns:
        RoomInfo: Room object containing name, color, position
    """
    return [room for room in info if room.name==name][0]

def get_room_info_by_color(color):
    """Returns room object, that has specified color

    Args:
        color (string): color of the room

    Returns:
        RoomInfo: Room object containing name, color, position
    """
    return [room for room in info if room.color==color][0]

def is_color_known(color):
    """Is the room with a ball of the specified color known? (do we know its position)

    Args:
        color (string): Color of the ball

    Returns:
        bool: Do we know the position of the ball with specified color (/that room)?
    """
    info_element = [room for room in info if room.color==color][0]
    return info_element.positions_known()

