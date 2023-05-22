import time
import random 
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


class Eddie_laserscan(Node):
    def __init__(self):
        super().__init__('eddie_laserscan')
        
        #state 1 means moving forward
        #state 0 means stop
        #state -1 means moving backward
        
        self.pub = self.create_publisher(Twist, '/model/eddiebot/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.scann = LaserScan()

        self.call_timer = self.create_timer(2.0, self._timer_cb)

    def listener_callback(self, msg):
        # print(len(msg.ranges)) #len is 2019 from 0-360
        # self.get_logger().info('Listener received: state is"%d"' % self.state)
        
            self.scann.ranges = msg.ranges[0:72]

            allMore = True
            for i in msg.ranges:
                if i < 0.4:
                    allMore = False #a wall detected
                    break

            if abs(time.time() - self.pivot_begin_time) > 2.0 or self.first_time:
                if not allMore:
                    self.state = -1
                    self.get_logger().info('Wall detected, state is"%d"' % self.state)
                    self.pivot_begin_time = time.time()
                    self.first_time = False
                else:
                    self.state = 1


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
