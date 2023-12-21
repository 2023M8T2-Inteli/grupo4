import time
import rclpy
from std_msgs.msg import Header
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from publisher import Publisher  # Substitua 'your_package_name' pelo nome real do seu pacote


class BatteryPercentage(Node):
    def __init__(self):
        super().__init__('battery_percentage')
        self.battery = Publisher(self, 'battery_state', '/battery_state', BatteryState)
        self.send_battery_percentage()

    def send_battery_percentage(self):
        while True:
            battery_msg = BatteryState(
                header=self.get_msg_header(),
                voltage=12.0,
                current=0.0,
                charge=0.0,
                capacity=0.0,
                design_capacity=0.0,
                percentage=0.0,
                power_supply_status=BatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
                power_supply_health=BatteryState.POWER_SUPPLY_HEALTH_GOOD,
                power_supply_technology=BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO,
                present=True,
                cell_voltage=[],
                location="",
                serial_number=""
            )
            self.battery.publish(battery_msg)
            time.sleep(1)

    def get_msg_header(self):
        # Substitua isso com a lógica real para obter o cabeçalho da mensagem
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'battery_frame'
        return header


def main(args=None):
    rclpy.init(args=args)
    battery_node = BatteryPercentage()
    rclpy.spin(battery_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
