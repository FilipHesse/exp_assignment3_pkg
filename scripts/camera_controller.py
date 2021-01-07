#! /usr/bin/env python3
"""File contains node CameraController

For details see class description
"""
import roslib
import rospy
import actionlib

from exp_assignment3_pkg.msg import EmptyAction
from std_msgs.msg import Float64
from math import pi


class CameraController:
    """This ROS continuously controlls the camera link
    
    The camera controller continuously publishes the angle 
    zero to the camera_position_controller, which is hadled by
    the controller manager.
    If the action look_left_right is called, the callback
    makes the robot look to the left (45°), then wait 2
    seconds, then look to the right (45°), wait again for
    two seconds, look to the center
    The action is preemptable after the waiting time of 2 
    seconds in the middle

        Attributes: 
            server (actionlib.SimpleActionServer): Action server
                for the action look_left_right
            pub (rospy.Publisher): Publisher for angle command
                of the camera_position_controller (controller
                manager)
    """
    def __init__(self):
        """Initializes attributes and calls function to publish zero
        """
        self.server = actionlib.SimpleActionServer(
            'look_left_right', EmptyAction, self.execute, False)
        self.server.start()
        self.pub = rospy.Publisher(
            "camera_position_controller/command", Float64, queue_size=10)

        self.action_active = False

        self.publish_continuously_zero()

    def publish_continuously_zero(self):
        """Publishes zero in a loop, so the controller always has a target value
        """
        r = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():
            if not self.action_active:
                self.pub.publish(Float64(0))
                r.sleep()

    def execute(self, goal):
        """Callback for Action server look_left_right

        Looks to the left, waits 2 seconds, looks 
        to the right, waits 2 seconds, looks to the center
        """
        self.action_active = True
        self.pub.publish(Float64(pi/4))
        rospy.sleep(2)
        if self.server.is_preempt_requested():
            self.action_active = False
            self.pub.publish(Float64(0))
            self.server.set_preempted()
            return
        self.pub.publish(Float64(-pi/4))
        rospy.sleep(2)
        self.pub.publish(Float64(0))
        self.server.set_succeeded()
        self.action_active = False

if __name__ == '__main__':
    """Entry point of script
    """
    rospy.init_node('camera_controller')
    server = CameraController()
    rospy.spin()
