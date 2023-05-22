import time
import random 
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

import cv2
import numpy as np


class Eddie_laserscan(Node):
    def __init__(self):
        super().__init__('eddie_laserscan')
        
        #state 1 means moving forward
        #state 0 means stop
        #state -1 means moving backward
        
        self.pub = self.create_publisher(Twist, '/model/eddiebot/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Image,
            '/kinect_rgbd_camera/image',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.img = Image()

        # self.call_timer = self.create_timer(2.0, self._timer_cb)

    def listener_callback(self, msg):
        self.get_logger().info(f"recieved image:  {msg.width}x{msg.height}, {msg.encoding}")    
        
        # Convert ROS Image message to OpenCV image
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # Define color ranges for red, yellow, and green
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        lower_green = np.array([60, 100, 100])
        upper_green = np.array([70, 255, 255])
        # Threshold the image to extract the colors
        mask_red = cv2.inRange(img, lower_red, upper_red)
        mask_yellow = cv2.inRange(img, lower_yellow, upper_yellow)
        mask_green = cv2.inRange(img, lower_green, upper_green)
        # Count the number of pixels for each color
        count_red = cv2.countNonZero(mask_red)
        count_yellow = cv2.countNonZero(mask_yellow)
        count_green = cv2.countNonZero(mask_green)
        # Determine which color has the most pixels
        if count_red > count_yellow and count_red > count_green:
            self.get_logger().info("Red is the dominant color")
        elif count_yellow > count_red and count_yellow > count_green:
            self.get_logger().info("Yellow is the dominant color")
        elif count_green > count_red and count_green > count_yellow:
            self.get_logger().info("Green is the dominant color")
        else:
            self.get_logger().info("No dominant color found")


    def _timer_cb(self):
        
        move_direction = Twist()

        #move forward
        if (self.state == 1):
            move_direction.angular.x = 0.0
            move_direction.angular.y = 0.0
            move_direction.angular.z = 0.0
            move_direction.linear.x = 2.0
            move_direction.linear.y = 0.0
            move_direction.linear.z = 0.0
            self.get_logger().info('publishing, state is"%d"' % self.state)
            # self.state= 1


        #move backward
        elif(self.state == -1):
            move_direction.angular.x = 0.0
            move_direction.angular.y = 0.0
            move_direction.angular.z = 0.0
            move_direction.linear.x = -2.0
            move_direction.linear.y = 0.0
            move_direction.linear.z = 0.0
            self.get_logger().info('publishing, state is"%d"' % self.state)
            self.state=2            


        #no movement
        # elif(self.state == 0):
        #     move_direction.angular.x = 0.0
        #     move_direction.angular.y = 0.0
        #     move_direction.angular.z = 0.0
        #     move_direction.linear.x = 0.0
        #     move_direction.linear.y = 0.0
        #     move_direction.linear.z = 0.0
        #     self.get_logger().info('publishing, state is"%d"' % self.state)
        #     self.state=2            


        #turn right
        elif(self.state == 2):
            move_direction.angular.x = 0.0
            move_direction.angular.y = 0.0
            move_direction.angular.z = -2.0
            move_direction.linear.x = 0.0
            move_direction.linear.y = 0.0
            move_direction.linear.z = 0.0
            self.get_logger().info('publishing, state is"%d"' % self.state)
            self.state=1            

        self.pub.publish(move_direction)
    
def main(args=None):    
    rclpy.init(args=args)
    eddie_laserscan = Eddie_laserscan()
    eddie_laserscan.state = 1
    eddie_laserscan.first_time = True
    eddie_laserscan.pivot_begin_time = time.time()
    rclpy.spin(eddie_laserscan)


    eddie_laserscan.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
