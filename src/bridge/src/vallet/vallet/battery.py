import rclpy
from typing import Any
from rclpy.node import Node
from vallet_msgs.msg import Log
from .streamer import Streamer
from .publisher import Publisher
from .subscriber import Subscriber
from .__init__ import socket_client
from sensor_msgs.msg import BatteryState


class BatteryPercentage(Node):
    def __init__(self):
        super().__init__("battery_percentage")
        self.logger = Publisher(self,
                                "log",
                                "/logs",
                                Log)
        self.battery = Subscriber(self,
                                  "battery_state",
                                  "/battery_state",
                                  BatteryState)
        self.streamer = Streamer(self,
                                 socket_client,
                                 "/battery",
                                 None,
                                 self.battery,
                                 self.listener_callback)
        
    def listener_callback(self, battery: BatteryState) -> None:
        battery_percentage = (battery.voltage - 11)/1.6 * 100

        log_msg = Log(node_name=self.battery.topic_name,
                      action=f'{battery_percentage}',
                      unix_time=int(self.get_clock().now().to_msg().sec))
        self.logger.publish(log_msg)

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