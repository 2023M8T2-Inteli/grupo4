#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose


class MyPublisher(Node):

    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(Pose, 'chatbot_msgs', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Pose()
        msg.position.x = float(input('Enter x position: '))
        msg.position.y = float(input('Enter y position: '))
        msg.position.z = float(input('Enter z position: '))
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Pose: x={}, y={}, z={}'.format(msg.position.x, msg.position.y, msg.position.z))


def main(args=None):
    rclpy.init(args=args)
    my_publisher = MyPublisher()
    rclpy.spin(my_publisher)
    my_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()