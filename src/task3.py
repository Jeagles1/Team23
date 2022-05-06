#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi
from sensor_msgs.msg import LaserScan
import numpy as np

class Maze:
    def callback_function(self, odom_data):
        # obtain the orientation and position co-ords:
        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')
        
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw 

        if self.startup:
            self.startup = False
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    def scan_callback(self, scan_data):
        """
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.min_distance = front_arc.min()
        self.object_angle = self.arc_angles[np.argmin(front_arc)]
        """
        self.left_distance = scan_data.ranges[90]
        self.right_distance = scan_data.ranges[270]
        self.front_distance = scan_data.ranges[0]
        self.backright_distance = scan_data.ranges[225]
        self.backleft_distance = scan_data.ranges[135]

    def __init__(self):
        node_name = "maze_nav"
        
        self.startup = True
        self.leftturn = False
        self.rightturn = False
        self.fullturn = False
        self.minorleft = False
        self.minorright = False

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.sub2 = rospy.Subscriber('odom', Odometry, self.callback_function)

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(10) # hz

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        self.left_distance = 0.0
        self.right_distance = 0.0
        self.front_distance = 0.0
        self.backleft_distance = 0.0
        self.backright_distance = 0.0
        
        self.vel = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node has been initialised...")

    def shutdownhook(self):
        self.pub.publish(Twist())
        self.ctrl_c = True
    
    def print_stuff(self, a_message):
        print(a_message)
        #print(f"current velocity: lin.x = {self.vel.linear.x:.1f}, ang.z = {self.vel.angular.z:.1f}")
        #print(f"current odometry: x = {self.x:.3f}, y = {self.y:.3f}, theta_z = {self.theta_z:.3f}")
        print(f"current scan: left = {self.left_distance:.3f}, front = {self.front_distance:.3f}, right = {self.right_distance:.3f}")

    def main_loop(self):
        status = ""
        wait = 0
        while not self.ctrl_c:
            if self.startup:
                self.vel = Twist()
                status = "init"
            elif self.leftturn:
                print("turning left")
                self.vel.linear.x = 0
                if (self.backright_distance < 0.2):
                    self.vel.linear.x = 0.1
                self.lastx = self.x
                self.lasty = self.y
                if abs(self.theta_z0 - self.theta_z) >= pi/2:
                    self.vel.angular.z = 0
                    self.leftturn = False
                else:
                    self.vel.angular.z = 0.4
            elif self.rightturn:
                print("turning right")
                self.vel.linear.x = 0
                if (self.backleft_distance < 0.2):
                    self.vel.linear.x = 0.1
                self.lastx = self.x
                self.lasty = self.y
                if abs(self.theta_z0 - self.theta_z) >= pi/2:
                    self.vel.angular.z = 0
                    self.rightturn = False
                else:
                    self.vel.angular.z = -0.4
            elif self.minorleft: #TODO: minorleft and minorright should be based on forward left and right angles on their distance, not for second option
                print("minorleft")
                self.vel.linear.x = 0.03
                if (self.backright_distance < 0.2):
                    self.vel.linear.x = 0.07
                if abs(self.theta_z0 - self.theta_z) >= pi/10:
                    self.vel.angular.z = 0
                    self.minorleft = False
                else:
                    self.vel.angular.z = 0.2
            elif self.minorright:
                print("minorright")
                self.vel.linear.x = 0.03
                if (self.backleft_distance < 0.2):
                    self.vel.linear.x = 0.07
                if abs(self.theta_z0 - self.theta_z) >= pi/10:
                    self.vel.angular.z = 0
                    self.minorright = False
                else:
                    self.vel.angular.z = -0.2
            elif self.front_distance < 0.4:
                print("end, need to turn")
                self.theta_z0 = self.theta_z
                if self.left_distance > 0.5:
                    self.leftturn = True
                elif self.right_distance > 0.5:
                    self.rightturn = True
                else:
                    # to do a full turn, do two right turns
                    self.rightturn = True
            elif self.left_distance > 0.5 and self.front_distance > 1 and (abs(self.lastx - self.x) > 0.3 or abs(self.lasty - self.y) > 0.3):
                print("can make left here")
                print(wait)
                if wait >= 10:
                    self.theta_z0 = self.theta_z
                    self.leftturn = True
                    wait = 0
                else:
                    wait += 1
            elif (self.left_distance < 0.4 and self.right_distance < 0.4 and self.right_distance - self.left_distance > 0.5) or self.right_distance < 0.2:
                self.theta_z0 = self.theta_z
                self.minorleft = True
            elif (self.left_distance < 0.4 and self.right_distance < 0.4 and self.left_distance - self.right_distance > 0.5) or self.left_distance < 0.2:
                self.theta_z0 = self.theta_z
                self.minorright = True
            else:
                print("forward")
                self.vel.linear.x = 0.1
            """
            elif self.turn:
                if abs(self.theta_z0 - self.theta_z) >= pi/2 and wait > 5:
                    # If the robot has turned 90 degrees (in radians) then stop turning
                    self.turn = False
                    self.vel = Twist()
                    self.theta_z0 = self.theta_z
                    status = "turn-fwd transition"
                    wait = 0
                else:
                    self.vel = Twist()
                    self.vel.angular.z = 0.2
                    status = "turning"
                    wait += 1
            else:
                if sqrt(pow(self.x0 - self.x, 2) + pow(self.y0 - self.y, 2)) >= 0.5:
                    # if distance travelled is greater than 0.5m then stop, and start turning:
                    self.vel = Twist()
                    self.turn = True
                    self.x0 = self.x
                    self.y0 = self.y
                    status = "fwd-turn transition"
                else:
                    self.vel = Twist()
                    self.vel.linear.x = 0.1
                    status = "moving forwards"
            """
            self.pub.publish(self.vel)
            self.print_stuff(status)
            self.rate.sleep()

if __name__ == '__main__':
    navmaze_instance = Maze()
    try:
        navmaze_instance.main_loop()
    except rospy.ROSInterruptException:
        pass