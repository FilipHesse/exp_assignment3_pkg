class RoomConfiguration:
    def __init__(self, color, room):
        self.color = color
        self.room = room 
        self.position = None

cfg=[RoomConfiguration("blue", "entrance"),
    RoomConfiguration("red", "closet"),
    RoomConfiguration("green", "living_room"),
    RoomConfiguration("yellow", "kitchen"),
    RoomConfiguration("orange", "bathroom"),
    RoomConfiguration("black", "bedroom")]