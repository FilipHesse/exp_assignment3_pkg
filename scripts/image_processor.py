#!/usr/bin/env python3
""" ROS Node, that processes the camera image by detecting the ball

Find further details in class description.  
""" 

import rospy

import cv2
import imutils
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from std_msgs.msg import Bool
from std_msgs.msg import Int64
from std_msgs.msg import String
from exp_assignment3_pkg.msg import BallCenterRadius
from exp_assignment3_pkg.msg import BallVisible

class ColoredBall:
    def __init__(self, color_str, lower_bound, upper_bound):
        self.color_str = color_str
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.radius = 0
        self.center = [0,0]
        self.visible= False


class ImageProcessor:
    """ImageProcessor node, that does the following things

    - Detects the ball with its centroid and contour
    - Publishes, if the ball is visible (camera1/ball_visible)
    - Publishes the processed camera image with the marked contour
        and centroid of the ball
    - Publishes the ball center and radius (camera1/ball_center_radius)
    """
    def __init__(self):
        """Initializes publishers and subscribers and CvBridge
        """
        self.counter = 0
        self.pub_img = rospy.Publisher("camera1/image_processed", Image, queue_size=1)
        self.pub_vis = rospy.Publisher("camera1/ball_visible", BallVisible, queue_size=1)
        self.pub_cr = rospy.Publisher( "camera1/ball_center_radius", BallCenterRadius, queue_size=1)
        self.number_subscriber = rospy.Subscriber("camera1/image_raw", Image, self.callback_raw_image)

        self.bridge = CvBridge()
        
    def comp_ball_center_radius(self, hsv_img, ball):
        mask = cv2.inRange(hsv_img, ball.lower_bound, ball.upper_bound)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        ball.center = [0, 0]
        ball.radius = 0
        ball.visible = False

                # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), ball.radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            ball.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if ball.radius > 10:
                ball.visible = True


    def callback_raw_image(self, msg):
        """ Callback: new image came is -> Process it

        After detecting the ball, three topics are published:
            camera1/image_processed
            camera1/ball_visible
            camera1/ball_center_radius
        """
        self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        colored_balls = [   ColoredBall('green',(50, 50, 20),(70, 255, 255)),
                            ColoredBall('yellow',(25, 50, 20),(32, 255, 255)),
                            ColoredBall('red',(0, 50, 100),(12, 255, 255)),
                            ColoredBall('blue',(105, 50, 20),(135, 255, 255)),
                            ColoredBall('pink',(143, 50, 20),(160, 255, 255)),
                            ColoredBall('black',(0, 0, 0),(179, 255, 10))]
        
        blurred = cv2.GaussianBlur(self.image, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        for ball in colored_balls:
            self.comp_ball_center_radius(hsv, ball)

        biggest_ball = max(colored_balls, key=lambda x : x.radius)
        if biggest_ball.visible:
            cv2.circle(self.image, biggest_ball.center, int(biggest_ball.radius),
                        (0, 255, 255), 2)
            cv2.circle(self.image, biggest_ball.center, 5, (0, 0, 255), -1)


        self.pub_img.publish((self.bridge.cv2_to_imgmsg(self.image, 'bgr8')))
        
        vis = BallVisible()
        vis.visible = Bool(biggest_ball.visible)
        vis.color = String(biggest_ball.color_str)
        self.pub_vis.publish(vis)

        cr = BallCenterRadius()
        cr.visible = Bool(biggest_ball.visible)
        cr.center_x = Int64(biggest_ball.center[0])
        cr.center_y = Int64(biggest_ball.center[1])
        cr.radius = Int64(int(biggest_ball.radius))
        self.pub_cr.publish(cr)


if __name__ == "__main__":
    """Main function of this script
    """
    rospy.init_node('image_processor')
    ImageProcessor()
    rospy.spin()

