import rospy

class RoomInfo:
    def __init__(self, color, name, r, g, b, x=None, y=None):
        self.color = color
        self.name = name 
        self.x = x
        self.y = y
        self.r = r
        self.g = g
        self.b = b
    
    def positions_known(self):
        if self.x == None or self.y == None:
            return False
        else:
            return True

info=[RoomInfo("blue", "entrance", 0., 0., 1.),
    RoomInfo("red", "closet", 1., 0., 0.),
    RoomInfo("green", "living_room", 0., 1., 0.),
    RoomInfo("yellow", "kitchen", 1., 1., 0.),
    RoomInfo("pink", "bathroom", 1., 0., 1.),
    RoomInfo("black", "bedroom", 0., 0., 0.)]

# info=[RoomInfo("blue", "entrance", -0.9, 7.9),
#     RoomInfo("red", "closet", -5, 2),
#     RoomInfo("green", "living_room"),
#     RoomInfo("yellow", "kitchen"),
#     RoomInfo("orange", "bathroom"),
#     RoomInfo("black", "bedroom")]


def get_room_info_by_name(name):
    return [room for room in info if room.name==name][0]

def get_room_info_by_color(color):
    return [room for room in info if room.color==color][0]

def is_color_known(color):
    #rospy.loginfo(f"Color: {color}")
    info_element = [room for room in info if room.color==color][0]
    return info_element.positions_known()

