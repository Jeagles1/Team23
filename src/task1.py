#!/usr/bin/env python3

import math
from xml.etree.ElementTree import PI
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Circle:

    def __init__(self):
        # When setting up the publisher, the "cmd_vel" topic needs to be specified
        # and the Twist message type needs to be provided
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('task1', anonymous=True)
        self.rate = rospy.Rate(1) # hz
        self.yaw = 0
        self.linear_x = 0
        self.linear_y = 0
        self.sub = rospy.Subscriber("odom", Odometry, self.callback_function)

        self.odom_data = Odometry()
        self.vel_cmd = Twist()
        self.pub.publish()
        self.angular_z = 0.0
        self.turn = 1
        self.loops = 0
        self.lasttime = rospy.get_rostime()
        
        self.lasttimeran = self.lasttime.secs
        
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo("the 'task 1' node is active...")

    def callback_function(self, odom_data):
        
        self.angular_z = odom_data.pose.pose.orientation.z

        angular_x = odom_data.pose.pose.orientation.x
        angular_y = odom_data.pose.pose.orientation.y
        angular_w = odom_data.pose.pose.orientation.w
        self.linear_x = odom_data.pose.pose.position.x
        self.linear_y = odom_data.pose.pose.position.y
        (roll, pitch, self.yaw) = euler_from_quaternion([angular_x, angular_y,self.angular_z, angular_w], "sxyz")
        self.yaw = self.yaw/(2*math.pi) *360
       
            
    def shutdownhook(self):
        self.vel_cmd.linear.x = 0.0 # m/s
        self.vel_cmd.angular.z = 0.0 # rad/s

        print("stopping the robot")

        # publish to the /cmd_vel topic to make the robot stop
        self.pub.publish(self.vel_cmd)

        self.ctrl_c = True

    def main_loop(self):
        
        while not self.ctrl_c and self.loops<2:
            # specify the radius of the circle:
            path_rad = 0.5 # m
            # linear velocity must be below 0.26m/s:
            lin_vel = 0.1 # m/s
            self.lasttime = rospy.get_rostime()
            
           
            if self.angular_z > -0.05 and self.angular_z < 0.05 and (self.lasttime.secs-self.lasttimeran) > 5:
                self.turn = self.turn*-1
                self.lasttimeran = self.lasttime.secs
                self.loops += 1


            if (self.yaw>=0):
                print("x= {:.2f} m,  y= {:.2f} m, yaw= {:.2f} degrees.".format(self.linear_x,self.linear_y,self.yaw))
            else:
                print("x= {:.2f} m,  y= {:.2f} m, yaw= {:.2f} degrees.".format(self.linear_x,self.linear_y,(360+self.yaw)))

            
            self.vel_cmd.linear.x = lin_vel
            self.vel_cmd.angular.z = (lin_vel * self.turn) / path_rad # rad/s

            self.pub.publish(self.vel_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    vel_ctlr = Circle()
    try:
        vel_ctlr.main_loop()
    except rospy.ROSInterruptException:
        pass