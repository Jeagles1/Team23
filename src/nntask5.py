#!/usr/bin/env python3
from ast import Num
from std_msgs.msg import String
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import degrees, pi
from enum import Enum
from random import randint
from math import sqrt
import roslaunch
import argparse
from pathlib import Path
import os
from sensor_msgs.msg import LaserScan

import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

parent = os.path.split(os.path.split(__file__)[0])[0]

class State(Enum):
    
    #Init States, left turning, right turning, straight forward, stop then turn around
    LT = 1
    RT = 2
    SW = 3
    TURNROUND = 4

class task5:

    #Init range value
    ranges = None
    #Init function


class State(Enum):
    
    #Init States, left turning, right turning, staight forward, stop and turn around
    LT = 1
    RT = 2
    SW = 3
    TURNROUND = 4

class task5:

    
    #Init range value
    ranges = None

    
    #Init function
    def __init__(self):
        rospy.init_node('task5')
        self.posx = 0.0
        self.posy = 0.0
        self.yaw  = 0.0


        self.photo = 0

        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_convert)
        self.publisher  = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel_cmd    = Twist()


        self.rate            = rospy.Rate(10)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        cli = argparse.ArgumentParser(description=f"Command-line interface for the node.")
        cli.add_argument("-colour",metavar="COL", type=String, default="blue")
        
        self.args = cli.parse_args(rospy.myargv()[1:])

        self.camera_subscriber = rospy.Subscriber("/camera/color/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()
        self.m00 = 0
        self.m00_min = 100000
        self.target_colour = self.args.colour.data

        self.takenpic = False

        self.colours = ["Blue", "Red", "Green", "Yellow"]
        self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (25,150,100)]
        self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (35,255,255)]

        self.lasttimeran = rospy.get_time()
        self.curtime = rospy.get_time()

        self.best = 0

        rospy.on_shutdown(self.shutdown_ops)
        self.main()


    
    #Function to round number in two decimal places
    def round(self, value, precision):
        value = int(value * (10**precision))

        return float(value) / (10**precision)

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, _ = cv_img.shape
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        # create a single mask to accommodate all four dectection colours:
        for i in range(4):
            if i == 0:
                mask = cv2.inRange(hsv_img, self.lower[i], self.upper[i])
            else:
                mask = mask + cv2.inRange(hsv_img, self.lower[i], self.upper[i])


        if (self.target_colour == "blue"):
            mask2 = cv2.inRange(hsv_img, self.lower[0], self.upper[0])
        if (self.target_colour == "red"):
            mask2 = cv2.inRange(hsv_img, self.lower[1], self.upper[1])
        if (self.target_colour == "green"):
            mask2 = cv2.inRange(hsv_img, self.lower[2], self.upper[2])
        if (self.target_colour == "yellow"):
            mask2 = cv2.inRange(hsv_img, self.lower[3], self.upper[3])
        m = cv2.moments(mask)
        m2 = cv2.moments(mask2)

        self.m00 = m["m00"]
        self.cy = m["m10"] / (m["m00"] + 1e-5)

        self.m200 = m2["m00"]
        self.cy2 = m2["m10"] / (m2["m00"] + 1e-5)

       
        if self.m00 > self.m00_min  :
            cv2.circle(cv_img, (int(self.cy), 200), 10, (0, 0, 255), 2)   

        cv2.imshow("crop", cv_img)
        if(self.photo == 0):
            self.save_image(cv_img, "the_beacon")
            print("Start Photo")
            self.photo = 1
        elif(self.photo == 1 and self.cy > 0):
            print("Beacon Photo")
            self.photo = 2
            self.save_image(cv_img, "the_beacon")
        elif((self.photo == 1 or self.photo == 2) and self.cy2 > self.best and self.cy2 <= width / 2):
            print("The Beacon Photo")
            self.photo = 2
            self.best = self.cy2
            self.save_image(cv_img, "the_beacon")

        cv2.waitKey(1)



    #Function for data publishing
    def publish(self):
        self.publisher.publish(self.vel_cmd)

    #Function of convert data
    def odom_convert(self, odom_data):
        orientation = odom_data.pose.pose.orientation
        position    = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')

        self.yaw  = self.round(degrees(yaw), 4)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)

    def scan_callback(self, scan_data):
        self.ranges = scan_data.ranges

    
    #Function to round number
    def round(self, value, precision):
        value = int(value * (10**precision))

        return float(value) / (10**precision)

    
   # Function for moving speed setting
    def move_speed_setting(self, linear = 0.0, angular = 0.0):
        self.vel_cmd.linear.x  = linear * (2 / 3)
        self.vel_cmd.angular.z = angular * -1

    def save_image(self, img, img_name):
        print("yo hoe")
        full_image_path = parent + "/snaps/" + (f"{img_name}.jpg")

        cv2.imwrite(str(full_image_path), img)
        print(f"Saved an image to '{full_image_path}'\n")


    #Function for data publishing
    def publish(self):
        self.publisher.publish(self.vel_cmd)

    
    #the Function of state calculation
    def state_cal(self, r):
        forward  = r[0]
        left  = r[-90]
        right  = r[90]
        backward  = r[180]
        front_right = r[55]
        front_left = r[-55]

        alpha           = 0.4
        side_threshold  = 0.7
        front_threshold = 0.6
        back_threshold  = 0.8

        r_prime    = np.array(r)
        close_count = float(np.count_nonzero(r_prime<=back_threshold)) / 360.0 * 100.0

        if (close_count > 75 and forward <= front_threshold and front_right <= alpha * 1.5):
            return State.TURNROUND
        elif (forward <= front_threshold and left <= side_threshold and right <= side_threshold and front_left <= alpha * 1.5 and front_right <= alpha * 1.5):
            return State.TURNROUND
        elif (forward <= front_threshold and backward <= back_threshold and right <= side_threshold and front_left <= alpha * 1.5 and front_right <= alpha * 1.5):
            return State.TURNROUND
        elif (forward <= front_threshold and left <= side_threshold and right <= side_threshold and front_left >= alpha * 1.5 and front_right <= alpha * 1.5):
            return State.LT
        elif (right <= side_threshold and front_right >= alpha * 1.5):
            return State.RT

        

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
            self.curtime = rospy.get_time()
            state = self.state_cal(self.ranges)
            if (state == State.SW):
                front_right = self.ranges[55]
                e  = 0.3 - front_right
                kp = 3
                self.move_speed_setting(0.25, kp *e)
            elif (state == State.RT):
                self.move_speed_setting(0.25, -0.9)

            elif (state == State.LT):
                self.move_speed_setting(0.25, 1.5)

            elif (state == State.TURNROUND):
                self.move_speed_setting(0,1.5)
            elif (state == State.RT or state == State.RT):
                self.move_speed_setting(0.25, -0.9)

            elif (state == State.LT or state == State.LT):
                self.move_speed_setting(0.25, 1.5)

            elif (state == State.TURNROUND):
                self.move_speed_setting(0, 1.5)

            else:
                self.move_speed_setting(0, 0)

            print(self.curtime)
            print(self.lasttimeran)
            if (self.curtime - self.lasttimeran >= 20):
                map_path = parent + "/maps/task5_map"

                launch = roslaunch.scriptapi.ROSLaunch()
                launch.start()

                print(f"Saving map at time: {rospy.get_time()}...")
                node = roslaunch.core.Node(package="map_server",
                                            node_type="map_saver",
                                            args=f"-f {map_path}")
                process = launch.launch(node)
                self.lasttimeran = self.curtime

            self.publish()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        task5()
    except rospy.ROSInterruptException:
        pass