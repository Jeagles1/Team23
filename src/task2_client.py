#! /usr/bin/python3
"""
   Autor:team23
"""
import rospy
import actionlib
import numpy as np
from math import degrees
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import cos, sin, atan, pi, sqrt
from random import randint, random, choice
from tf.transformations import euler_from_quaternion
from com2009_msgs.msg import SearchAction,SearchGoal,SearchFeedback

class client:

    ranges  = None
    closest_object_angle = None

   #Init function
    def __init__(self):
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel_cmd   = Twist()

        self.posx = 0.0
        self.posy = 0.0
        self.yaw  = 0.0

        self.subscriber      = rospy.Subscriber('/odom', Odometry, self.odom_convert)
        self.action_complete = False

        rospy.init_node('search_action_client')
        self.rate = rospy.Rate(5)
        self.goal = SearchGoal()

        self.client = actionlib.SimpleActionClient('/search_action_server', SearchAction)
        self.client.wait_for_server()
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback) # Creat subscriber for laser data

        rospy.on_shutdown(self.shutdown)
        self.main()

    def set_cmd_vel(self, linear = 0.0, angular = 0.0):

        self.vel_cmd.linear.x  = linear
        self.vel_cmd.angular.z = angular

    def publish(self):
        self.publisher.publish(self.vel_cmd)

    def stop_move(self):
        self.set_cmd_vel()
        self.publish()
   
    #Callback function for laser data received
    def scan_callback(self, data):
        self.ranges =data.ranges

    
    #Send Function for goal sending to action server 
    def send_goal(self, distance, velocity):
        self.goal.fwd_velocity = velocity
        self.goal.approach_distance = distance
        self.client.send_goal(self.goal, feedback_cb = self.feedback_callback) # Send goal

    
    #Callback function for any feedbacks from action server
    def feedback_callback(self, feedback_data):
        #Avoid going too far and wasting time
        if feedback_data.current_distance_travelled >= 2:
            self.client.cancel_goal()
            print("Current moving stoped")

    def round_number(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)

    def odom_convert(self, odom_data):
        orientation = odom_data.pose.pose.orientation
        position    = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')

        self.posx = self.round_number(position.x, 4)
        self.posy = self.round_number(position.y, 4)
        self.yaw  = self.round_number(degrees(yaw), 4)

    
    #Adjust direction to avoid being hit 
    def adjust_direction(self):
        thre_value            = 1.5
        left_degree_distance  = np.array(self.ranges[0:89])
        right_degree_distance = np.array(self.ranges[270:359])

        
        #If condition for Determine which direction to turn to adjust    
        if left_degree_distance.max() >= thre_value and right_degree_distance.max() >= thre_value:
            if self.closest_object_angle < 0:
                turn_velocity = -0.40
            else:
                turn_velocity = 0.40

        else:

            if right_degree_distance.max() > left_degree_distance.max():
                turn_velocity = -0.40
            else:
                turn_velocity = 0.40

        self.set_cmd_vel(0, turn_velocity)
        self.publish()

        
        #If condition for determine when to stop adjusting 
        if turn_velocity < 0:
            while np.array(self.ranges[0:20]).min() <= thre_value:
                continue

        else:

            while np.array(self.ranges[-21:]).min() <= thre_value:
                continue

        self.stop_move()

    
   #Shutdown function
    def shutdown(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            self.stop_move()
            rospy.logwarn("Goal Cancelled")

    
    #Main function for robot control
    def main(self):
        while True:
            self.action_complete = False
            self.send_goal(distance = 0.5, velocity = 0.26)
            #Do while loop when action has not been complete
            while self.client.get_state() < 2:
                self.rate.sleep()

            result = self.client.get_result()
            self.closest_object_angle = result.closest_object_angle

            self.adjust_direction() # Turn 20 degrees
            self.action_complete = True
            print("Adjust direction!")

if __name__ == '__main__':
    try:
        client()
    except rospy.ROSInterruptException:
        pass