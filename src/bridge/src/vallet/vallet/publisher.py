from typing import Any
from rclpy.node import Node


class Publisher:
    def __init__(self, node: Node, name: str, topic_name: str, topic_type: any):
        self.node = node
        self.name = name
        self.publisher = None
        self.topic_type = topic_type
        self.topic_name = topic_name

        self.create_pub()

        self.node.get_logger().info(f"Publisher {self.name} created.")

    def create_pub(self) -> None:
        self.publisher = self.node.create_publisher(
            self.topic_type, self.topic_name, 1000
        )

        self.node.get_logger().info(f"Publisher {self.name} connected.")

    def create_timer(self, period: float, timer_callback: Any) -> None:
        self.node.create_timer(period, timer_callback)

        self.node.get_logger().info(f"Timer on {self.name} node created.")

    def publish(self, message: any) -> None:
        self.publisher.publish(message)
        self.node.get_logger().info(
            f"Publishing {message} on {self.topic_name}")
