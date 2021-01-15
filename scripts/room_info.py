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