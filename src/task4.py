#! /usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
from random import randint
from turtle import distance
import numpy as np
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from math import degrees, pi
from enum import Enum

from sensor_msgs.msg import LaserScan
# Import some image processing modules:
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
    BEACON = 6
    FINDCOLOUR = 7
    END = 8
    ZE = 9
    RM = 10


class colour_search(object):

    ranges = None

    def __init__(self):
        node_name = "turn_and_face"

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback_function)
        self.publisher  = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.init_node(node_name)

        self.move_rate = "slow" # fast, slow or stop
        self.stop_counter = 0

        self.odom_data = Odometry()
        self.vel_cmd = Twist()

        self.pub.publish()
        
        self.start_yaw = 0

        self.posx = 0.0
        self.posy = 0.0
        self.yaw  = 0.0
        self.startup = True
        self.crop_x = 0
        self.crop_y = 0
        self.state = State.SW

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        self.targetturn = 0
        self.targetdirection = 0

        self.lasttime = rospy.get_rostime()
        self.lasttimeran = 0

        self.rate            = rospy.Rate(10)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)
        
        self.m00 = 0
        self.m00_min = 100000
        self.target_colour = ""

        # Thresholds for ["Blue", "Red", "Green", "Turquoise", "Purple","Yellow"]
        self.colours = ["Blue", "Red", "Green", "Turquoise", "Purple","Yellow"]
        self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (75, 150, 100),(145,205,100),(25,150,100)]
        self.upper = [(130, 255, 255), (20, 255, 255), (70, 255, 255), (100, 255, 255),(155,255,255),(35,255,255)]

        self.state = State.FINDCOLOUR

    def distance_to_Beacon(self, r):
        forward  = r[0]
        return forward

    def distance_to_LeftWall(self, r):
        left  = r[55]
        return left
    
    def distance_to_RightWall(self, r):
        right  = r[-55]
        return right

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

    def shutdown_ops(self):
        self.vel_cmd.angular.z = 0
        self.vel_cmd.linear.x = 0
        self.pub.publish(self.vel_cmd)
        cv2.destroyAllWindows()
        self.ctrl_c = True

    #When entering the random movement state
    def enteringCompleteRandom(self):
        self.theta_z0 = self.yaw
        self.targetturn = randint(90, 180)
        self.targetdirection = randint(0,1)
        if self.targetdirection == 0:
            self.targetdirection = -1

    def enteringInformedRandom(self):
        self.theta_z0 = self.yaw
        # highest_a = np.argmax(self.ranges)
        rangestuff = np.where(np.array(self.ranges) < 1, 10, np.array(self.ranges))
        lowest_a = np.argmin(rangestuff)
        if lowest_a > 180:
            self.targetdirection = -2.5
        else:
            self.targetdirection = 2.5
        if lowest_a == 180:
            self.targetturn = 180
        else:
            self.targetturn = lowest_a % 180

    def callback_function(self, odom_data):
        angular_x = odom_data.pose.pose.orientation.x
        angular_y = odom_data.pose.pose.orientation.y
        angular_w = odom_data.pose.pose.orientation.w
        angular_z = odom_data.pose.pose.orientation.z
        (roll, pitch, self.yaw) = euler_from_quaternion([angular_x, angular_y, angular_z,angular_w], "sxyz")
        self.yaw = self.yaw/(2*math.pi) *360
        orientation = odom_data.pose.pose.orientation
        position    = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')

        self.yaw  = self.round(degrees(yaw), 4)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)
    
    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, _ = cv_img.shape
        crop_width = width - 1500
        crop_height = 50
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))
        

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # create a single mask to accommodate all four dectection colours:
        for i in range(6):
            if i == 0:
                mask = cv2.inRange(hsv_img, self.lower[i], self.upper[i])
            else:
                mask = mask + cv2.inRange(hsv_img, self.lower[i], self.upper[i])

        m = cv2.moments(mask)
        
        self.m00 = m["m00"]
        
        self.cy = m["m10"] / (m["m00"] + 1e-5)

        cv2.imshow("cropped image", crop_img)
        
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        self.lasttime = rospy.get_rostime() 
        if (self.lasttimeran == 0):
                self.lasttimeran = self.lasttime.secs
                
        print(hsv_img[25][25])
        """add code to make it only do this after x seconds just in case"""
        for i in range(6):
            if(self.lower[i][0]<=hsv_img[25][25][0] and self.upper[i][0]>=hsv_img[25][25][0] and self.lower[i][1]<=hsv_img[25][25][1] and self.upper[i][1]>=hsv_img[25][25][1] and self.target_colour == "" and (self.lasttime.secs - self.lasttimeran >= 6)):
                self.target_colour = (self.colours[i])
                self.state = State.RM
                print("Target colour is:" + self.target_colour)
                # Thresholds for ["Blue", "Red", "Green", "Turquoise", "Purple","Yellow"]
                self.target_colour = "Red"
                print("Target colour is:" + self.target_colour)

        for i in range(6):
            if(self.lower[i][0]<=hsv_img[25][25][0] and self.upper[i][0]>=hsv_img[25][25][0] and self.lower[i][1]<=hsv_img[25][25][1] and self.upper[i][1]>=hsv_img[25][25][1] and self.target_colour != "" and (self.lasttime.secs - self.lasttimeran >= 10)):

                if (self.target_colour == (self.colours[i]) and self.cy >= 80 and self.cy <= 120):
                    print("Found beacon")
                    self.vel_cmd.linear.x = 0
                    self.vel_cmd.angular.z = 0
                    self.pub.publish(self.vel_cmd)
                    self.state = State.BEACON
                    
                

        """cv2.imshow("colour", filtered_img)"""
        cv2.waitKey(1)

    def publish(self):
        self.publisher.publish(self.vel_cmd)

    def main(self):
        self.lasttime = rospy.get_rostime()
        
        while not self.ctrl_c and not (self.ranges) :
            self.rate.sleep()        
        while not self.ctrl_c and (self.state == State.FINDCOLOUR):    
            self.rate.sleep()

            self.vel_cmd.angular.z = 0.5
            self.pub.publish(self.vel_cmd)

        
                 
        
        while not self.ctrl_c and (self.state != State.FINDCOLOUR) and self.move_rate != "stop":
            if (self.state == State.BEACON):
                self.pub.publish(self.vel_cmd)
                
                if (self.cy >= 80 and self.cy <= 100):
                    self.vel_cmd.angular.z = 0
                elif (self.cy<80 or self.distance_to_RightWall(self.ranges) <= 0.4):
                    print("Turning left")
                    self.vel_cmd.angular.z = 0.4
                elif (self.distance_to_LeftWall(self.ranges) <= 0.4 or self.cy>100):
                    print("Turning right")
                    self.vel_cmd.angular.z = -0.4

                if (self.distance_to_Beacon(self.ranges)<0.45):
                # blob detected
                    self.move_rate = "stop"
                        
                    
                    
                else:
                    self.move_rate = "slow"
                if self.move_rate == "fast":
                    self.vel_cmd.linear.x = 0.3
                elif self.move_rate == "slow":
                    print("Slow")
                    self.vel_cmd.linear.x = 0.2
                elif self.move_rate == "stop":    
                    print("Stop")                
                    self.vel_cmd.linear.x = 0.2
                else:
                    self.vel_cmd.linear.x = 0.2
                self.pub.publish(self.vel_cmd)
                
            
            else:
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
            minsides = np.amin(np.append(l,r))
            print(minleft - minright)
            front_left = np.amin(np.array(self.ranges[35:45]))
            front_right = np.amin(np.array(self.ranges[315:325]))
            mid_left = np.amin(np.array(self.ranges[45:55]))
            mid_right = np.amin(np.array(self.ranges[305:315]))
            if (self.state == State.ZE):
                # Zone Exploring
                if (front_left <= 0.26 or front_right <= 0.26 or front <= 0.23):
                    # About to hit a wall
                    if ((front_left - front_right <= 0.1 and front_left - front_right >= -0.1) or (front_left >= 0.7 and front_right >= 0.7)):
                        # If it thinks its stuck when moving away, try turning, possibly change this to random
                        #self.move_speed_setting(-0.05, (left - right + 0.2) * 0.5 * (1 / forward))
                        print("Stuck")

                        self.enteringCompleteRandom()
                        #self.enteringInformedRandom()
                        self.state = State.RM
                    #elif ((front_left <= 0.22 or front_right <= 0.22) and front > 0.3):
                        #self.move_speed_setting(forward * 0.005, 0)
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
                        self.move_speed_setting((forward * 0.005), (((left * mid_left) - (right * mid_right)) * 1.5 * (1 / (forward + 1)) * minsides))
            elif (self.state == State.RM):
                # Random Movement
                if abs(self.theta_z0 - self.yaw) >= ((self.targetturn / 180) * pi):
                    # Stop and go back to zone exploration
                    self.move_speed_setting(0, 0)
                    self.state = State.ZE
                else:
                    # Keep Turning
                    self.move_speed_setting(-0.05, self.targetdirection)

            self.publish()
            self.rate.sleep()

        
            
            
if __name__ == "__main__":
    search_instance = colour_search()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass