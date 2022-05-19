#! /usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import numpy as np
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from math import degrees
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
        self.posx = 0.0
        self.posy = 0.0
        self.yaw = 0
        self.start_yaw = 0

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
        self.target_colour = "Turquoise"

        # Thresholds for ["Blue", "Red", "Green", "Turquoise"]
        self.colours = ["Blue", "Red", "Green", "Turquoise"]
        self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (75, 150, 100)]
        self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (100, 255, 255)]

        self.state = State.BEACON

    #Function to round number
    def round(self, value, precision):
        value = int(value * (10**precision))

        return float(value) / (10**precision)

   # Function for moving speed setting
    def move_speed_setting(self, linear = 0.0, angular = 0.0):
        self.vel_cmd.linear.x  = linear
        self.vel_cmd.angular.z = angular

    def state_cal(self, r):
        forward  = r[0]
        left  = r[90]
        right  = r[-90]
        backward  = r[180]
        front_right = r[-55]
        front_left = r[55]

        alpha           = 0.6
        side_threshold  = 0.5
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
            return State.LD
        elif (right <= side_threshold and front_right >= alpha * 1.5):
            return State.RD

        

        elif (right >= side_threshold and front_right >= alpha * 1.5):
            return State.RT

        elif (forward <= front_threshold and left >= side_threshold and right <= side_threshold and front_left >= alpha * 1.5 and front_right <= alpha * 1.5):
            return State.LT

        else:
            return State.SW

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
        crop_width = width - 800
        crop_height = 200
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


        if self.m00 > self.m00_min  :
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
          
        cv2.imshow("cropped image", crop_img)
        
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        self.lasttime = rospy.get_rostime() 
        # print(hsv_img[0][0])
        if (self.lasttimeran == 0):
                self.lasttimeran = self.lasttime.secs
                
           
        """add code to make it only do this after x seconds just in case"""
        for i in range(4):
            if(self.lower[i][0]<=hsv_img[0][0][0] and self.upper[i][0]>=hsv_img[0][0][0] and self.lower[i][1]<=hsv_img[0][0][1] and self.upper[i][1]>=hsv_img[0][0][1] and self.target_colour == "" and (self.lasttime.secs - self.lasttimeran >= 9)):
                self.target_colour = (self.colours[i])
                self.state = State.END
                print("Target colour is:" + self.target_colour)

        for i in range(4):
            if(self.lower[i][0]<=hsv_img[0][0][0] and self.upper[i][0]>=hsv_img[0][0][0] and self.lower[i][1]<=hsv_img[0][0][1] and self.upper[i][1]>=hsv_img[0][0][1] and self.target_colour != "" and (self.lasttime.secs - self.lasttimeran >= 15)):
                if (self.target_colour == (self.colours[i])):
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
            if(self.target_colour!=""):
                print(self.target_colour)

            self.vel_cmd.angular.z = 0.5
            self.pub.publish(self.vel_cmd)

        
                 
        
        while not self.ctrl_c and (self.state != State.FINDCOLOUR) and self.move_rate != "stop":
            if (self.state == State.BEACON):
                self.pub.publish(self.vel_cmd)
                print(self.move_rate)  
                print(self.cy)
                if self.m00 > self.m00_min*5:
                # blob detected
                    if self.cy >= 505 and self.cy <= 550:
                        if self.move_rate == "slow":
                            self.move_rate = "stop"
                        
                        
                else:
                    self.move_rate = "fast"
                if self.move_rate == "fast":
                    self.vel_cmd.linear.x = 0.3
                elif self.move_rate == "slow":
                    self.vel_cmd.linear.x = 0.2
                elif self.move_rate == "stop":                    
                    self.vel_cmd.linear.x = 0.2
                else:
                    self.vel_cmd.linear.x = 0.2
                self.pub.publish(self.vel_cmd)
            else:
                self.state = self.state_cal(self.ranges)

                if (self.state == State.SW):
                    front_right = self.ranges[-55]
                    e  = 0.4 - front_right
                    kp = 3
                    self.move_speed_setting(0.25, kp * e)

                elif (self.state == State.RD or self.state == State.RT):
                    self.move_speed_setting(0.25, -0.9)

                elif (self.state == State.LD or self.state == State.LT):
                    self.move_speed_setting(0.25, 1.5)

                elif (self.state == State.END):
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