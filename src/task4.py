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

   
        
        self.m00 = 0
        self.m00_min = 100000
        self.target_colour = ""

        # Thresholds for ["Blue", "Red", "Green", "Turquoise", "Purple","Yellow"]
        self.colours = ["Blue", "Red", "Green", "Turquoise", "Purple","Yellow"]
        self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (75, 150, 100),(145,205,100),(25,150,100)]
        self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (100, 255, 255),(155,255,255),(35,255,255)]

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
        self.vel_cmd.linear.x  = linear 
        self.vel_cmd.angular.z = angular * -1


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
            return State.END
        elif (forward <= front_threshold and left <= side_threshold and right <= side_threshold and front_left <= alpha * 1.5 and front_right <= alpha * 1.5):
            return State.END
        elif (forward <= front_threshold and backward <= back_threshold and right <= side_threshold and front_left <= alpha * 1.5 and front_right <= alpha * 1.5):
            return State.END
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
        crop_width = 1000
        crop_height = 50
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))
        

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        if (self.target_colour == "Red"):
            mask = cv2.inRange(hsv_img, self.lower[1], self.upper[1])
            m = cv2.moments(mask)
        
            self.m00 = m["m00"]
        
            self.cy = m["m10"] / (m["m00"] + 1e-5)

        """
        # create a single mask to accommodate all four dectection colours:
        for i in range(6):
            if i == 0:
                mask = cv2.inRange(hsv_img, self.lower[i], self.upper[i])
            else:
                mask = mask + cv2.inRange(hsv_img, self.lower[i], self.upper[i])

        m = cv2.moments(mask)
        
        self.m00 = m["m00"]
        
        self.cy = m["m10"] / (m["m00"] + 1e-5)
        """

        cv2.imshow("cropped image", crop_img)
        
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        self.lasttime = rospy.get_rostime() 
        if (self.lasttimeran == 0):
                self.lasttimeran = self.lasttime.secs
                
        # print(hsv_img[0][0])
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
                print(self.colours[i])
                print(self.cy)
                #if (self.target_colour == (self.colours[i]) and self.cy >= 80 and self.cy <= 120):
                if (self.target_colour == (self.colours[i])):
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
                    self.vel_cmd.angular.z = 0.8
                elif (self.distance_to_LeftWall(self.ranges) <= 0.4 or self.cy>100):
                    print("Turning right")
                    self.vel_cmd.angular.z = -0.8

                if (self.distance_to_Beacon(self.ranges)<0.45):
                # blob detected
                    self.move_rate = "stop"
                        
                    
                    
                else:
                    self.move_rate = "slow"
                if self.move_rate == "fast":
                    self.vel_cmd.linear.x = 0.3
                elif self.move_rate == "slow":
                    print("Slow")
                    self.vel_cmd.linear.x = 0
                elif self.move_rate == "stop":    
                    print("Stop")                
                    self.vel_cmd.linear.x = 0.2
                else:
                    self.vel_cmd.linear.x = 0.2
                self.publish()
                
            
            else:
                state = self.state_cal(self.ranges)
            if (np.amin(np.append(self.ranges[0:30], self.ranges[330:360])) <= 0.4 and (self.ranges[55] >= 0.4 or self.ranges[-55] >= 0.4)):
                self.move_speed_setting(0, 1)
            elif (state == State.SW):
                front_right = self.ranges[55]
                e  = 0.4 - front_right
                kp = 3
                self.move_speed_setting(0.25, kp *e)
            elif (state == State.RT):
                self.move_speed_setting(0.25, -0.9)

            elif (state == State.LT):
                self.move_speed_setting(0.25, 1.5)

            elif (state == State.END):
                self.move_speed_setting(0,1.5)
            elif (state == State.RT or state == State.RT):
                self.move_speed_setting(0.25, -0.9)

            elif (state == State.LT or state == State.LT):
                self.move_speed_setting(0.25, 1.5)

            elif (state == State.END):
                self.move_speed_setting(0, 1.5)

            else:
                self.move_speed_setting(0, 0)

            self.publish()
            self.rate.sleep()

        
            
            
if __name__ == "__main__":
    search_instance = colour_search()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass