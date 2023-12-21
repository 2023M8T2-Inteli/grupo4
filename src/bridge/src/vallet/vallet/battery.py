import rclpy
from rclpy.node import Node
from vallet_msgs.msg import Log
from .streamer import Streamer
from .subscriber import Subscriber
from .__init__ import socket_client
from sensor_msgs.msg import BatteryState


class BatteryPercentage(Node):
    def __init__(self):
        super().__init__("battery_percentage")
        self.battery = Subscriber(self,
                                  "battery_state",
                                  "/battery_state",
                                  BatteryState,
                                  1)
        self.streamer = Streamer(self,
                                 socket_client,
                                 "/battery",
                                 None,
                                 self.battery,
                                 self.listener_callback)

    def listener_callback(self, battery: BatteryState) -> None:
        battery_percentage = (battery.voltage - 11)/1.6 * 100

        self.streamer.emit_event(battery_percentage)

        self.get_logger().info(
            f'Receiving battery percentage: {battery_percentage}')


def main(args=None):
    rclpy.init(args=args)
    battery_node = BatteryPercentage()
    rclpy.spin(battery_node)
    battery_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
