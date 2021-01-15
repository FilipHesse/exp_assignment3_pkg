import rospy

class RoomInfo:
    def __init__(self, color, name, x=None, y=None):
        self.color = color
        self.name = name 
        self.x = x
        self.y = y

# info=[RoomInfo("blue", "entrance"),
#     RoomInfo("red", "closet"),
#     RoomInfo("green", "living_room"),
#     RoomInfo("yellow", "kitchen"),
#     RoomInfo("orange", "bathroom"),
#     RoomInfo("black", "bedroom")]

info=[RoomInfo("blue", "entrance", -0.9, 7.9),
    RoomInfo("red", "closet", -5, 2),
    RoomInfo("green", "living_room"),
    RoomInfo("yellow", "kitchen"),
    RoomInfo("orange", "bathroom"),
    RoomInfo("black", "bedroom")]


def get_room_info_by_name(name):
    return [room for room in info if room.name==name][0]

def is_color_known(color):
    rospy.loginfo(f"Color: {color}")
    info_element = [room for room in info if room.color==color][0]
    if info_element.x == None or info_element.y == None:
        return False
    else:
        return True
