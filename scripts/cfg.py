class Configuration:
    def __init__(self, color, room):
        self.color = color
        self.room = room 
        self.position = None

cfg=[Configuration("blue", "entrance"),
    Configuration("red", "closet"),
    Configuration("green", "living_room"),
    Configuration("yellow", "kitchen"),
    Configuration("orange", "bathroom"),
    Configuration("black", "bedroom")]