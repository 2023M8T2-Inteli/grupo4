import os
import rclpy
from vallet_msgs.msg import Log
from rclpy.node import Node
from .subscriber import Subscriber
from ament_index_python.packages import get_package_share_directory


class Logger(Node):
    def __init__(self):
        super().__init__("logger")
        self.package_name = os.path.basename(os.path.dirname(__file__))
        self.sub = Subscriber(self, "log", "/logs", Log)
        self.sub.create_sub(self.log_callback)

    def log_callback(self, msg: Log):
        self.get_logger().info(
            f"Received: {msg.node_name} - {msg.action} - {msg.unix_time}")
        content_to_add = f'[{msg.node_name}] - {msg.action} [{msg.unix_time}] \n'
        self.get_logger().info(f"Adding content: {content_to_add}")
        with open(self._get_file_path(), 'a') as file:
            file.write(content_to_add)

    def _create_file(self, data_file_path: str) -> None:
        """Create an empty file."""
        self.get_logger().info(f"Creating file: {data_file_path}")
        with open(data_file_path, 'w') as _:
            pass

    def _get_file_path(self) -> str:
        """Get the full path of the data file."""
        self.get_logger().info("Getting file path")
        package_share_directory = get_package_share_directory(
            self.package_name)
        assets_directory = os.path.join(package_share_directory, 'assets')

        if not os.path.exists(assets_directory):
            self.get_logger().info(
                f"Creating assets directory: {assets_directory}")
            os.makedirs(assets_directory, exist_ok=True)

        data_file_path = os.path.join(assets_directory, 'temp_data.txt')

        if not os.path.exists(data_file_path):
            self.get_logger().info(f"File not found")
            self._create_file(data_file_path)

        return data_file_path


def main(args=None):
    rclpy.init(args=args)
    logger = Logger()
    rclpy.spin(logger)
    logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
