#!/usr/bin/env python3
"""File contains node BallFollower

For details see class description
"""
import rospy
from geometry_msgs.msg import Twist, Pose
from exp_assignment3_pkg.msg import BallCenterRadius
from nav_msgs.msg import Odometry
from exp_assignment3_pkg.msg import EmptyAction, EmptyGoal
import actionlib
from actionlib_msgs.msg import GoalStatus


class BallFollower:
    """BallFollower class makes the robot follow the ball

    ...by publishing appropriate velocity commands. Image based visual servoing
    is implemented: the node knows from a subscribed topic
    (camera1/ball_center_radius) the position and the size of the ball in the
    image frame. From this information the node computes an angular and linear
    velocity such that the ball will be located in the image center with a
    specified size (-> distance)

    This node contains 4 ROS communication interfaces:
        pub_cmd_vel: Publisher to topic "cmd_vel"
        sub_ball: Subscriber to topic "camera1/ball_center_radius"
        server: Action server "follow_ball" to activate ball following
            (can be cancelled)
    """
    def __init__(self):
        """Init members for computation
        """

        # Ball visibility, position, radius
        self.ball_visible = False
        self.ball_center_x = 0
        self.ball_center_y = 0
        self.ball_radius = 0

        # Publisher, Subscriber, ActionServer, ActionClient
        self.pub_cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        self.sub_ball = rospy.Subscriber(
            "camera1/ball_center_radius", BallCenterRadius, self.callback_ball_center_radius)

        self.server = actionlib.SimpleActionServer(
            'follow_ball', EmptyAction, self.callback_follow_ball, False)
        self.server.start()

    def callback_ball_center_radius(self, msg):
        """callback to update ball position and radius
        """
        self.ball_visible = msg.visible.data
        self.ball_center_x = msg.center_x.data
        self.ball_center_y = msg.center_y.data
        self.ball_radius = msg.radius.data

    def callback_follow_ball(self, goal):
        """Action callback, when follow_ball is activated

        This function runs for a long time, as long as the action
        is active. 

        It computes the angular and linear velocities to make the
        ball appear in the image center.

        Args:
            goal (EmptyGoal): unused but required
        """
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            # Only do something, if ball_follower active and ball visible
            if self.ball_visible:
                vel = Twist()
                # Rotate to get ball into center of image
                if abs(self.ball_center_x-400) > 10:  # We are not properly aligned
                    #rospy.loginfo("Not aligned")
                    vel.angular.z = 0.004*(400 - self.ball_center_x)

                desired_radius = 105
                # We are aligned but too far or too close
                if abs(self.ball_center_x-400) < 40 and abs(desired_radius - self.ball_radius) > 10:
                    #rospy.loginfo("Wrong distance")
                    vel.linear.x = 0.01*(desired_radius - self.ball_radius)
                    #rotate very slow
                    vel.angular.z = 0.0005*(400 - self.ball_center_x)

                # We are aligned and close enough
                if abs(self.ball_center_x-400) < 30 and abs(desired_radius - self.ball_radius) <= 10:
                    vel.linear.x = 0
                    vel.angular.z = 0
                    self.pub_cmd_vel.publish(vel)
                    self.server.set_succeeded(True)
                    return

                self.pub_cmd_vel.publish(vel)

            else:  # If we dont see tha ball, dont move
                vel = Twist()  # Zero twist
                self.pub_cmd_vel.publish(vel)

            if self.server.is_preempt_requested():
                vel = Twist()
                self.pub_cmd_vel.publish(vel)
                self.server.set_preempted()
                return

            r.sleep()


if __name__ == '__main__':
    """Entry point for script
    """
    rospy.init_node('ball_follower')
    BallFollower()
