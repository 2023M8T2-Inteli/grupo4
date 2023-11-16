import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from battery.subscriber import Subscriber


class BatteryPercentage(Node):
    def __init__(self):
        super().__init__("battery_percentage")

        self.battery = Subscriber(
            self, "battery_state", "/battery_state", BatteryState)
        self.battery.create_sub(self.listener_callback)

    def listener_callback(self, battery: BatteryState) -> None:
        battery_percentage = (battery.voltage - 11)/1.6 * 100
        self.get_logger().info(
            f'Receiving battery state: {battery_percentage}')
        
def main(args=None):
    rclpy.init(args=args)
    battery_node = BatteryPercentage()
    rclpy.spin(battery_node)
    battery_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()