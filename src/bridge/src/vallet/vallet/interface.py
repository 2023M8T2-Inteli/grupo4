import rclpy
from rclpy.node import Node
from .publisher import Publisher
from geometry_msgs.msg import Pose, Point


class Interface(Node):
    def __init__(self):
        super().__init__("interface")
        self.oracle = Publisher(self, "enqueue", "/enqueue", Pose)
        self.oracle.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Pose(position=Point(x=float(input("X: ")), y=float(input("Y: ")), z=0.0))

        self.oracle.publish(msg)

        self.get_logger().info(f"Pose {msg.position.x, msg.position.y} published.")


def main(args=None):
    rclpy.init(args=args)
    interface_node = Interface()
    rclpy.spin(interface_node)
    interface_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
