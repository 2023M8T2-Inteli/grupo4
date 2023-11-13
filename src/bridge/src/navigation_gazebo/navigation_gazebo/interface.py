import rclpy
import logging
from rclpy.node import Node
from geometry_msgs.msg import Pose
from navigation_gazebo.publisher import Publisher


class Interface(Node):
    def __init__(self):
        super().__init__("interface")
        self.oracle = Publisher(self, "enqueue", "/enqueue", Pose)
        self.oracle.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        print("Know thyself; tell me a position, and I will show you where you will go.")

        msg = Pose()

        msg.position.x = float(input('Enter x position: '))
        msg.position.y = float(input('Enter y position: '))
        msg.position.z = 0.0 # Zero because we don't want to rotate the robot

        self.oracle.publish(msg)

        self.get_logger().info(f"Pose {msg.position.x, msg.position.y} published.")

    def destroy(self):
        self.oracle.destroy_pub()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    interface_node = Interface()
    rclpy.spin(interface_node)
    interface_node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
