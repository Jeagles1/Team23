#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import degrees
from enum import Enum

from sensor_msgs.msg import LaserScan

import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

class State(Enum):
    
    #Init States, left turning, right turning,left distance, right distance, strait wall, end
    LT = 1
    RT = 2
    LD = 3
    RD = 4
    SW = 5
    END = 6

class task3:

    
    #Init range value
    ranges = None

    
    #Init function
    
    def __init__(self):
        rospy.init_node('task3')
        self.posx = 0.0
        self.posy = 0.0
        self.yaw  = 0.0

        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_convert)
        self.publisher  = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel_cmd    = Twist()


        self.rate            = rospy.Rate(10)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)


        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()
        self.m00 = 0
        self.m00_min = 100000
        self.target_colour = "Blue"

        self.colours = ["Blue", "Red", "Green", "Turquoise"]
        self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (75, 150, 100)]
        self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (100, 255, 255)]

        rospy.on_shutdown(self.shutdown_ops)
        self.main()

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 100
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # create a single mask to accommodate all four dectection colours:
        for i in range(4):
            if i == 0:
                mask = cv2.inRange(hsv_img, self.lower[i], self.upper[i])
            else:
                mask = mask + cv2.inRange(hsv_img, self.lower[i], self.upper[i])

        m = cv2.moments(mask)
        
        self.m00 = m["m00"]
        self.cy = m["m10"] / (m["m00"] + 1e-5)

        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        lower_threshold =  (115, 224, 100)
        upper_threshold = (130, 255, 255)
        img_mask = cv2.inRange(hsv_img, lower_threshold, upper_threshold)
        filtered_img_blue = cv2.bitwise_and(crop_img, crop_img, mask = img_mask)

        lower_threshold =  (25, 150, 100)
        upper_threshold = (70, 255, 255)
        img_mask = cv2.inRange(hsv_img, lower_threshold, upper_threshold)
        filtered_img_green = cv2.bitwise_and(crop_img, crop_img, mask = img_mask)

        lower_threshold =  (0, 185, 100)
        upper_threshold = (10, 255, 255)
        img_mask = cv2.inRange(hsv_img, lower_threshold, upper_threshold)
        filtered_img_red = cv2.bitwise_and(crop_img, crop_img, mask = img_mask)

        lower_threshold =  (75, 150, 100)
        upper_threshold = (100, 255, 255)
        img_mask = cv2.inRange(hsv_img, lower_threshold, upper_threshold)
        filtered_img_turquoise = cv2.bitwise_and(crop_img, crop_img, mask = img_mask)
       
        if self.m00 > self.m00_min  :
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)   

        cv2.imshow("crop", crop_img)
        cv2.waitKey(1)

    #Function to round number
    def round(self, value, precision):
        value = int(value * (10**precision))

        return float(value) / (10**precision)

    
   # Function for moving speed setting
    def move_speed_setting(self, linear = 0.0, angular = 0.0):
        self.vel_cmd.linear.x  = linear * (2 / 3)
        self.vel_cmd.angular.z = angular * (1 / 3)


    #Function for data publishing
    def publish(self):
        self.publisher.publish(self.vel_cmd)

    
    #the Function of state calculation
    def state_cal(self, r):
        forward  = r[0]
        left  =  np.amin(np.array(r[85:95]))
        right  = np.amin(np.array(r[265:275]))
        backward  = np.amin(np.array(r[175:185]))
        front_right = np.amin(np.array(r[300:310]))
        front_left = np.amin(np.array(r[45:65]))

        alpha           = 1
        side_threshold  = 0.2
        front_threshold = 1.5
        back_threshold  = 0.4

        r_prime    = np.array(r)
        close_count = float(np.count_nonzero(r_prime<=back_threshold)) / 360.0 * 100.0

        if (close_count > 75 and forward <= front_threshold and front_right <= alpha * 1.5):
            return State.END
        elif (forward <= front_threshold and left <= side_threshold and right <= side_threshold and front_left <= alpha * 1.5 and front_right <= alpha * 1.5):
            return State.END
        elif (forward <= front_threshold and backward <= back_threshold and right <= side_threshold and front_left <= alpha * 1.5 and front_right <= alpha * 1.5):
            return State.END
        elif (forward <= front_threshold and left <= side_threshold and right <= side_threshold and front_left >= alpha * 1.5 and front_right <= alpha * 1.5):
            return State.LD
        elif (right <= side_threshold and front_right >= alpha * 1.5):
            return State.RD

        

        elif (right >= side_threshold and front_right >= alpha * 1.5):
            return State.RT

        elif (forward <= front_threshold and left >= side_threshold and right <= side_threshold and front_left >= alpha * 1.5 and front_right <= alpha * 1.5):
            return State.LT

        else:
            return State.SW

    #Function of convert data
    def odom_convert(self, odom_data):
        orientation = odom_data.pose.pose.orientation
        position    = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')

        self.yaw  = self.round(degrees(yaw), 4)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)

    def stop(self):
        self.move_speed_setting()
        self.publish()

    def scan_callback(self, scan_data):
        self.ranges = scan_data.ranges

    
    #Shutdown function
    def shutdown_ops(self):
        rospy.logwarn("Received a shutdown request.")
        self.stop()

    
    #Main loop function
    def main(self):
        while not (self.ranges):
            self.rate.sleep()
        while True:
            state = self.state_cal(self.ranges)
            if (state == State.SW):
                front_right = self.ranges[-55]
                e  = 0.4 - front_right
                kp = 3
                self.move_speed_setting(0.25, kp * e)

            elif (state == State.RD or state == State.RT):
                self.move_speed_setting(0.25, -0.9)

            elif (state == State.LD or state == State.LT):
                self.move_speed_setting(0.25, 1.5)

            elif (state == State.END):
                self.move_speed_setting(0, 1.5)

            else:
                self.move_speed_setting(0, 0)

            self.publish()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        task3()
    except rospy.ROSInterruptException:
        pass