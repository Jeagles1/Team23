#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import degrees, pi
from enum import Enum
from random import randint

from sensor_msgs.msg import LaserScan

import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

class State(Enum):
    
    #Init States, zone explore, random movement
    ZE = 1
    RM = 2

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

        self.targetturn = 0
        self.targetdirection = 0

        self.state = State.ZE

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
        self.vel_cmd.linear.x  = linear * (3 / 4)
        self.vel_cmd.angular.z = angular * (1 / 3)


    #Function for data publishing
    def publish(self):
        self.publisher.publish(self.vel_cmd)

    
    #the Function of state calculation
    def state_cal(self, r):
        #forward  = r[0]
        forward = np.amin(np.append(np.array(r[0:30]),np.array(r[330:360])))
        left  =  np.amin(np.array(r[85:95]))
        right  = np.amin(np.array(r[265:275]))
        backward  = np.amin(np.array(r[175:185]))
        back_left = np.amin(np.array(r[125:145]))
        back_right = np.amin(np.array(r[215:235]))
        front_right = np.amin(np.array(r[295:335]))
        front_left = np.amin(np.array(r[25:65]))

        alpha           = 0.4
        side_threshold  = 0.4
        front_threshold = 0.5
        back_threshold  = 0.4

        r_prime    = np.array(r)
        close_count = float(np.count_nonzero(r_prime<=back_threshold)) / 360.0 * 100.0

        if (close_count > 75 and forward <= front_threshold and front_right <= alpha * 1.5):
            return State.END
        elif (forward <= front_threshold and left <= side_threshold and right <= side_threshold and front_left <= alpha * 1 and front_right <= alpha * 1):
            return State.END
        elif (forward <= front_threshold and backward <= back_threshold and right <= side_threshold and front_left <= alpha * 1 and front_right <= alpha * 1):
            return State.END
        #elif (forward <= front_threshold or front_right <= alpha * 0.5 or front_left <= alpha * 0.5):
            #return State.END

        #elif (forward <= front_threshold and left <= side_threshold and right <= side_threshold and front_left >= alpha * 1.5 and front_right <= alpha * 1.5):
            #return State.LD
        #elif (right <= side_threshold and front_right >= alpha * 1.5):
            #return State.RD

        
        #elif (right >= side_threshold and right >= left and front_right >= alpha * 1.5 and back_right <= alpha * 1.5):
            #return State.RD

        #elif (forward <= front_threshold):
            #if (left >= right or front_left >= front_right):
                #return State.LT
            #else:
                #return State.RT

        #elif (right >= side_threshold and right >= left and front_right >= alpha * 1.5):
            #return State.RT

        #elif (forward <= front_threshold and left >= side_threshold and front_left >= alpha * 1.5 and back_left <= alpha * 1.5):
            #return State.LD

        #elif (forward <= front_threshold and left >= side_threshold and front_left >= alpha * 1.5):
            #return State.LT

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

    #When entering the random movement state
    def enteringRandom(self):
        self.targetturn = randint(90, 180)
        self.targetdirection = randint(0,1)
        if self.targetdirection == 0:
            self.targetdirection = -1

    
    #Main loop function
    def main(self):
        while not (self.ranges):
            self.rate.sleep()
        while True:
            forward = np.sum(np.append(np.array(self.ranges[0:15]), np.array(self.ranges[345:360])))
            front = np.amin(np.append(np.array(self.ranges[0:30]), np.array(self.ranges[330:360])))
            left = np.sum(self.ranges[30:90])
            right = np.sum(self.ranges[270:330])
            l = self.ranges[30:90]
            l = l[l != 0]
            minleft = np.amin(l)
            #minleft = np.amin(self.ranges[30:90][self.ranges[30:90] != 0])
            #minright = np.amin(self.ranges[270:330][self.ranges[270:330 != 0]])
            r = self.ranges[270:330]
            r = r[r != 0]
            minright = np.amin(r)
            print(minleft - minright)
            front_left = np.amin(np.array(self.ranges[35:45]))
            front_right = np.amin(np.array(self.ranges[315:325]))
            if (self.state == State.ZE):
                # Zone Exploring
                if (front_left <= 0.28 or front_right <= 0.28 or front <= 0.25):
                    # About to hit a wall
                    if ((front_left - front_right <= 0.05 and front_left - front_right >= -0.05) or (front_left >= 0.7 and front_right >= 0.7)):
                        # If it thinks its stuck when moving away, try turning, possibly change this to random
                        self.move_speed_setting(-0.05, (left - right + 0.2) * 0.5 * (1 / forward))

                        #self.enteringRandom()
                        #self.state = State.RM
                    else:
                        # If not, just move back
                        self.move_speed_setting(-0.2, 0)
                else:
                    # If not about to hit a wall
                    if (minleft - minright <= 0.03 and minleft - minright >= -0.03):
                        # If it thinks its in a 'corridoor', then move just forward
                        print("yeah")
                        #self.move_speed_setting((forward * 0.005), 1 * (1 / forward))
                        self.move_speed_setting((forward * 0.005), 0)
                    else:
                        # If it is not in a corridoor, move based on zone in front of and around it
                        self.move_speed_setting((forward * 0.005), (left - right) * 1 * (1 / forward))
            elif (self.State == State.RM):
                # Random Movement
                if abs(self.theta_z0 - self.theta_z) >= ((self.targetdirection / 180) * pi):
                    # Stop and go back to zone exploration
                    self.state = State.ZE
                else:
                    # Keep Turning
                    self.move_speed_setting(0, self.targetdirection)

            """
            state = self.state_cal(self.ranges)
            back_left = np.amin(np.array(self.ranges[115:145]))
            back_right = np.amin(np.array(self.ranges[215:245]))
            front_right = np.amin(np.array(self.ranges[295:335]))
            front_left = np.amin(np.array(self.ranges[25:65]))
            corner_backs = np.amin(np.append(back_left, back_right))
            corner_fronts = np.amin(np.append(front_left, front_right))
            if (state == State.SW):
                front_right = self.ranges[-55]
                e  = 0.4 - front_right
                kp = 3
                self.move_speed_setting(0.25, kp * e)
                #self.move_speed_setting(0.25, 0)

            elif (state == State.RD):
                self.move_speed_setting(0.2, -0.9)

            elif (state == State.RT):
                self.move_speed_setting(0, -0.9)

            elif (state == State.LD):
                self.move_speed_setting(0.2, 0.9)

            elif (state == State.LT):
                self.move_speed_setting(0, 0.9)

            elif (state == State.END):
                self.move_speed_setting(-0.05, 1.5)

            else:
                self.move_speed_setting(0, 0)

            if (corner_backs <= 0.2 and corner_fronts >= 0.2):
                self.vel_cmd.linear.x = 0.1
            """
            self.publish()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        task3()
    except rospy.ROSInterruptException:
        pass