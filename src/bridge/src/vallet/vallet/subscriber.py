from typing import Any
from rclpy.node import Node


class Subscriber():
    def __init__(self,  node: Node, name: str, srv_name: str, srv_type: Any, custom_qos: Any = None):
        self.node = node
        self.name = name
        self.topic_type = srv_type
        self.topic_name = srv_name
        self.subscription = None
        self.custom_qos = custom_qos
        self.node.get_logger().info(f"Subscription {self.name} created.")

    def create_sub(self, subscription_callback: Any) -> None:
        self.subscription = self.node.create_subscription(
            self.topic_type, self.topic_name, subscription_callback, self.custom_qos if self.custom_qos is not None else 10
        )

        self.node.get_logger().info(f"Subscription {self.name} connected.")

    def create_timer(self, period: float, timer_callback: Any) -> None:
        self.node.create_timer(period, timer_callback)

        self.node.get_logger().info(f"Timer on {self.name} node created.")
