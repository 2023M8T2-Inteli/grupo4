import os
import rclpy
from logger.msg import Log
from rclpy.node import Node
from modules import Subscriber, Publisher, Streamer
from ament_index_python.packages import get_package_share_directory


class Test(Node):
    def __init__(self):
        super().__init__("logger")

        self.pub = Publisher(self, "log", "/logs", Log)

    def pub_loop(self):
        while True:
            self.get_logger().info(f"Publishing")
            pub_msg = Log()
            pub_msg.node_name = "test"
            pub_msg.action = "test"
            pub_msg.unix_time = 0
            self.pub.publish(pub_msg)
            self.get_logger().info(f"Published")



def main(args=None):
    rclpy.init(args=args)
    logger = Test()
    logger.pub_loop()
    rclpy.spin(logger)
    logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
