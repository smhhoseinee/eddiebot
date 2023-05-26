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
import time


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

        self.call_timer = self.create_timer(0.1, self._timer_cb)

    def listener_callback(self, msg):
        # self.get_logger().info(f"recieved image:  {msg.width}x{msg.height}, {msg.encoding}")    
    
    
        # Convert the ROS Image message to a numpy array
        image_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
        # self.get_logger().info(f"image_data shape:{image_data.shape}") 

        # Convert the image to the BGR color space (OpenCV uses BGR by default)
        bgr_image = cv2.cvtColor(image_data, cv2.COLOR_RGB2BGR)
        # self.get_logger().info(f"bgr_image shape:{bgr_image.shape}") 

        # Define the lower and upper bounds for each color (in BGR format)
        color_ranges = {
            'red': ([0, 0, 200], [20, 20, 255]),    # Example range for red color
            'yellow': ([0, 150, 150], [50, 255, 255]),    # Example range for yellow color
            'green': ([0, 150, 0], [50, 255, 50]),    # Example range for green color
            'blue': ([25, 0, 0], [255, 255, 255])
        }

        # Initialize counters for each color
        color_counts = {
            'red': 0,
            'yellow': 0,
            'green': 0,
            'blue': 0
        }

        if bgr_image[:, :, 0].max() < 25:
            if bgr_image[:, :, 1].max() < 25: # not green -> red
                if self.state not in [3,33]:
                    self.get_logger().info(f"red detected") 
                    self.state = 3
                    self.pivot_start = True

            elif bgr_image[:, :, 2].max() < 25: # not red -> green
                if self.state not in [4,44]:
                    self.get_logger().info(f"green detected") 
                    self.state = 4
                    self.pivot_start = True

            else: #not red & green -> yellow
                # self.get_logger().info(f"yellow") 
                if self.state not in [2,22]:
                    self.get_logger().info(f"yellow detected") 
                    self.state = 2
                    self.pivot_start = True
                    
            
            

        if self.state == 1 and self.pivot_start:
        
            # Iterate over each pixel in the image
            left_quarter_center_image = bgr_image[int(msg.height/4):int(msg.height* 3/4), int(msg.width/4):int(msg.width/2), 0]
            right_quarter_center_image = bgr_image[int(msg.height/4):int(msg.height* 3/4), int(msg.width/2):int(msg.width*3/4), 0]

            left_half_of_image = bgr_image[:, 0:int(msg.width/2), 0]
            right_half_of_image = bgr_image[:, int(msg.width/2):int(msg.width), 0]

            left_mid_line_of_image = left_half_of_image[int(msg.height/2), :]
            right_mid_line_of_image = right_half_of_image[int(msg.height/2), :]



            number_of_not_blue_left = 0
            number_of_not_blue_right = 0

            for pixel in left_mid_line_of_image:
                if pixel < 25:
                    number_of_not_blue_left += 1     

            for pixel in right_mid_line_of_image:
                if pixel < 25:
                    number_of_not_blue_right += 1     


            self.get_logger().info(f" bias_difference:{self.bias_difference}= {number_of_not_blue_right} - {number_of_not_blue_left} ")            
            self.bias_difference = number_of_not_blue_left - number_of_not_blue_right    



            # for row in center_image:
            #     for pixel in row:
            #         # Check if the pixel's color falls within any of the color ranges
            #         for color, (lower, upper) in color_ranges.items():
            #             # if cv2.inRange(np.array(pixel), np.array(lower), np.array(upper)):
            #             #     color_counts[color] += 1

            #             if np.all(np.logical_and(pixel >= lower, pixel <= upper)):
            #                 color_counts[color] += 1
            #                 # self.get_logger().info(f"color:{color} match pixel:{pixel}, lower:{lower}, upper:{upper}") 
                    

        # Determine the color that appears most frequently
        dominant_color = max(color_counts, key=color_counts.get)
        # self.get_logger().info(f"red pixels: {color_counts['red']}") 
        # self.get_logger().info(f"yellow color: {color_counts['yellow']}") 
        # self.get_logger().info(f"green color: {color_counts['green']}") 

        # self.get_logger().info(f"Dominant color: {dominant_color}") 



    def _timer_cb(self):
        
        move_direction = Twist()
        
        
        #move forward
        if (self.state == 1):
            move_direction.angular.x = 0.0
            move_direction.angular.y = 0.0
            move_direction.angular.z = self.pivot_rate * self.bias_difference 
            # move_direction.angular.z = 0.0 
            move_direction.linear.x = 0.5
            move_direction.linear.y = 0.0
            move_direction.linear.z = 0.0
            # self.get_logger().info('publishing, state is"%d"' % self.state)
            # self.state= 1


        # Yellow, 1-stop
        elif(self.state == 2):
            move_direction.angular.x = 0.0
            move_direction.angular.y = 0.0
            move_direction.angular.z = 0.0
            move_direction.linear.x = 0.0
            move_direction.linear.y = 0.0
            move_direction.linear.z = 0.0
            # self.get_logger().info('publishing, state is"%d"' % self.state)
            
            self.state=22
            self.rotation_begin_time = int(round(time.time() * 1000))            

        # Yellow, 2-turn right
        elif(self.state == 22):
            move_direction.angular.x = 0.0
            move_direction.angular.y = 0.0
            move_direction.angular.z = -1.0
            move_direction.linear.x = 0.0
            move_direction.linear.y = 0.0
            move_direction.linear.z = 0.0
            #turn 90 degrees right
            current_time_in_miliseconds = int(round(time.time() * 1000))
            if (current_time_in_miliseconds - self.rotation_begin_time) < self.rotation_duration:
            # if (self.next_target_color != 0):
                self.state=22
                # self.get_logger().info('difference in time is"%d"' % (current_time_in_miliseconds - self.rotation_begin_time))
                
            else:
                self.state=1 # start moving forward again     
            # self.get_logger().info('publishing, state is"%d"' % self.state)


        # Red, 1-stop
        elif(self.state == 3):
            move_direction.angular.x = 0.0
            move_direction.angular.y = 0.0
            move_direction.angular.z = 0.0
            move_direction.linear.x = 0.0
            move_direction.linear.y = 0.0
            move_direction.linear.z = 0.0
            # self.get_logger().info('publishing, state is"%d"' % self.state)
            
            self.state=33
            self.rotation_begin_time = int(round(time.time() * 1000))            

        # Red, 2-turn left
        elif(self.state == 33):
            move_direction.angular.x = 0.0
            move_direction.angular.y = 0.0
            move_direction.angular.z = 1.0
            move_direction.linear.x = 0.0
            move_direction.linear.y = 0.0
            move_direction.linear.z = 0.0
            #turn 90 degrees left
            current_time_in_miliseconds = int(round(time.time() * 1000))
            if (current_time_in_miliseconds - self.rotation_begin_time) < self.rotation_duration:
            # if (self.next_target_color != 0):
                self.state=33
                # self.get_logger().info('difference in time is"%d"' % (current_time_in_miliseconds - self.rotation_begin_time))
                
            else:
                self.state=1 # start moving forward again     
            # self.get_logger().info('publishing, state is"%d"' % self.state)

        # Green, 1-stop
        elif(self.state == 4):
            move_direction.angular.x = 0.0
            move_direction.angular.y = 0.0
            move_direction.angular.z = 0.0
            move_direction.linear.x = 0.0
            move_direction.linear.y = 0.0
            move_direction.linear.z = 0.0
            # self.get_logger().info('publishing, state is"%d"' % self.state)
            
            self.state=44
            self.rotation_begin_time = int(round(time.time() * 1000))            

        # Green, 2-turn around
        elif(self.state == 44):
            move_direction.angular.x = 0.0
            move_direction.angular.y = 0.0
            move_direction.angular.z = 1.0
            move_direction.linear.x = 0.0
            move_direction.linear.y = 0.0
            move_direction.linear.z = 0.0
            #turn 90 degrees left
            current_time_in_miliseconds = int(round(time.time() * 1000))
            
            self.state=44
            # self.get_logger().info('difference in time is"%d"' % (current_time_in_miliseconds - self.rotation_begin_time))
                
            # self.get_logger().info('publishing, state is"%d"' % self.state)

        self.pub.publish(move_direction)
    
def main(args=None):    
    rclpy.init(args=args)
    eddie_laserscan = Eddie_laserscan()
    eddie_laserscan.state = 1
    eddie_laserscan.first_time = True
    eddie_laserscan.rotation_begin_time = time.time()
    # pivot_degree is a small pivot while moving to set direction closer to the target
    eddie_laserscan.pivot_rate = 0.001 
    eddie_laserscan.bias_difference = 0 
    eddie_laserscan.pivot_start = False 
    eddie_laserscan.next_target_color = 0
    eddie_laserscan.rotation_duration = 5300

    


    rclpy.spin(eddie_laserscan)


    eddie_laserscan.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
