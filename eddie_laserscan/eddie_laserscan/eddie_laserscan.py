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
            self.get_logger().info(f"no more blue now") 
            if bgr_image[:, :, 1].max() < 25: # no green
                self.get_logger().info(f"red") 
            elif bgr_image[:, :, 2].max() < 25: # no red
                self.get_logger().info(f"green") 
            else: #not red & green -> yellow
                # self.get_logger().info(f"yellow") 
                if self.state not in [2,22]:
                    self.state = 2
                    
            
            

        # Iterate over each pixel in the image
        # for row in bgr_image:
        #     for pixel in row:
        #         # Check if the pixel's color falls within any of the color ranges
        #         for color, (lower, upper) in color_ranges.items():
        #             # if cv2.inRange(np.array(pixel), np.array(lower), np.array(upper)):
        #             #     color_counts[color] += 1

        #             if np.all(np.logical_and(pixel >= lower, pixel <= upper)):
        #                 color_counts[color] += 1
        #                 # self.get_logger().info(f"color:{color} match pixel:{pixel}, lower:{lower}, upper:{upper}") 

                    #here we have to see whether there is no blue
                    

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
            move_direction.angular.z = 0.0
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
            self.get_logger().info('publishing, state is"%d"' % self.state)
            
            self.state=22
            self.pivot_begin_time = int(round(time.time() * 1000))            

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
            if (current_time_in_miliseconds - self.pivot_begin_time) < 5300:
                self.state=22
                self.get_logger().info('difference in time is"%d"' % (current_time_in_miliseconds - self.pivot_begin_time))
                
            else:
                self.state=1 # start moving forward again     
            self.get_logger().info('publishing, state is"%d"' % self.state)



        # Red, 1-stop
        elif(self.state == 3):
            move_direction.angular.x = 0.0
            move_direction.angular.y = 0.0
            move_direction.angular.z = 0.0
            move_direction.linear.x = 0.0
            move_direction.linear.y = 0.0
            move_direction.linear.z = 0.0
            self.get_logger().info('publishing, state is"%d"' % self.state)
            
            self.state=33
            self.pivot_begin_time = int(round(time.time() * 1000))            

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
            if (current_time_in_miliseconds - self.pivot_begin_time) < 5300:
                self.state=33
                self.get_logger().info('difference in time is"%d"' % (current_time_in_miliseconds - self.pivot_begin_time))
                
            else:
                self.state=1 # start moving forward again     
            self.get_logger().info('publishing, state is"%d"' % self.state)

     
     

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
