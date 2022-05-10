#! /usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import the tb3 modules (which needs to exist within the "week6_vision" package)


class colour_search(object):

    def __init__(self):
        node_name = "turn_and_face"

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback_function)

        rospy.init_node(node_name)

        self.odom_data = Odometry()
        self.vel_cmd = Twist()

        self.pub.publish()

        self.yaw = 0
        self.start_yaw = 0
        

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        
        self.move_rate = "" # fast, slow or stop
        self.stop_counter = 0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)
        
        self.m00 = 0
        self.m00_min = 100000
        self.target_colour = ""

        # Thresholds for ["Blue", "Red", "Green", "Turquoise"]
        self.colours = ["Blue", "Red", "Green", "Turquoise"]
        self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (75, 150, 100)]
        self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (100, 255, 255)]

    def shutdown_ops(self):
        self.vel_cmd.angular.z = 0
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
    
    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
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
    
        print(hsv_img[0][0])

    
        """add code to make it only do this after x seconds just in case"""
        for i in range(4):
            if(self.lower[i][0]<=hsv_img[0][0][0] and self.upper[i][0]>=hsv_img[0][0][0] and self.lower[i][1]<=hsv_img[0][0][1] and self.upper[i][1]>=hsv_img[0][0][1] and self.target_colour == ""):
                self.target_colour = (self.colours[i])


        """cv2.imshow("colour", filtered_img)"""
        cv2.waitKey(1)

    def main(self):
        while not self.ctrl_c:
            
           
            self.rate.sleep()
            if(self.target_colour!=""):
                print(self.target_colour)

            self.vel_cmd.angular.z = 0.5

                

            self.pub.publish(self.vel_cmd)

        """while not self.ctrl_c:
            if self.stop_counter > 0:
                self.stop_counter -= 1

            if self.m00 > self.m00_min:
                # blob detected
                if self.cy >= 560-100 and self.cy <= 560+100:
                    if self.move_rate == "slow":
                        self.move_rate = "stop"
                        self.stop_counter = 20
                else:
                    self.move_rate = "slow"
            else:
                self.move_rate = "fast"
                
            if self.move_rate == "fast":
                print(f"MOVING FAST: I can't see anything at the moment (blob size = {self.m00:.0f}), scanning the area...")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
            elif self.move_rate == "slow":
                print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            elif self.move_rate == "stop" and self.stop_counter > 0:
                print(f"STOPPED: The blob of colour is now dead-ahead at y-position {self.cy:.0f} pixels... Counting down: {self.stop_counter}")
                self.robot_controller.set_move_cmd(0.0, 0.0)
            else:
                print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            
            self.robot_controller.publish()"""
            
            
if __name__ == "__main__":
    search_instance = colour_search()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass