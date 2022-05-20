#! /usr/bin/python3
import os
import rospy
import actionlib
import numpy as np
import datetime as dt
from math import degrees
from math import radians, sqrt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from com2009_msgs.msg import SearchFeedback,SearchResult,SearchAction


class server(object):

    min_distance = None
    feedback = SearchFeedback()
    result = SearchResult()

    def __init__(self):

        self.actionserver = actionlib.SimpleActionServer("/search_action_server",SearchAction, self.server_launch, auto_start=False)
        self.actionserver.start()
        print("Task 2 server is ready and waiting...")

        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.publisher  = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel_cmd    = Twist()

        self.posx = 0.0
        self.posy = 0.0
        self.yaw  = 0.0

        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_convert)

    def set_vel(self, linear = 0.0, angular = 0.0):
        self.vel_cmd.linear.x  = linear
        self.vel_cmd.angular.z = angular

    def publish(self):
        self.publisher.publish(self.vel_cmd)

    def stop_move(self):
        self.set_vel()
        self.publish()

    #Callback function for laser data
    def scan_callback(self, scan_data):
        left_view   = scan_data.ranges[0:15]
        right_view  = scan_data.ranges[-16:]
        front_view  = np.array(left_view[::-1] + right_view[::-1])
        view_angles = np.arange(-16, 15)
        self.min_distance = front_view.min()
        self.object_angle = view_angles[np.argmin(front_view)]
    
    def odom_convert(self, odom_data):
        orientation = odom_data.pose.pose.orientation
        position    = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')

        self.yaw  = self.round_number(degrees(yaw), 4)
        self.posx = self.round_number(position.x, 4)
        self.posy = self.round_number(position.y, 4)
    
    def round_number(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)

    
    #Init function for this server
    def server_launch(self, goal):
        complete  = True
        velocity  = goal.fwd_velocity
        distance  = goal.approach_distance

        if velocity <= 0 or velocity > 0.26:
            print("Please provide a non-zero, positive velocity that is no larger than 0.26m/s.")
            complete = False

        if distance < 0.1 or distance > 1:
            print("Please provide a distance between 10cm and 1m.")
            complete = False

        if not complete:
            self.actionserver.set_aborted()
            return

        print("Request to move forwards at {}m/s, and to stop at {}m".format(velocity, distance))

        self.set_vel(velocity, 0) # set velocity
        
        #Do while loop when min_distance has not been created
        while not self.min_distance:
            pass

        x_init = self.posx # Refresh the x position
        y_init = self.posy # Refresh the y position

        
        #Do while loop when there are no obstacles around
        while self.min_distance >= (distance + 0.1):
            self.publish() # Keep moving forward

            if self.actionserver.is_preempt_requested():
                rospy.loginfo('Cancelling the search')
                self.actionserver.set_preempted()
                self.stop_move() # Stop moving forward
                complete = False
                break

            distance_travelled = sqrt((x_init - self.posx)**2 + (y_init - self.posy)**2)
            self.feedback.current_distance_travelled = self.round_number(distance_travelled, 2)
            self.actionserver.publish_feedback(self.feedback) # Publish feedback

        if complete:
            rospy.loginfo('Search process complete.')
            distance_travelled = sqrt((x_init - self.posx)**2 + (y_init - self.posy)**2)
            self.result.total_distance_travelled = self.round_number(distance_travelled, 2)
            self.result.closest_object_angle     = self.object_angle
            self.result.closest_object_distance  = self.min_distance

            self.actionserver.set_succeeded(self.result)
            self.stop_move()

if __name__ == '__main__':
    rospy.init_node('camera_sweep_action_server')
    server()
    rospy.spin()