import json
from typing import Any
from socketio import Client
from rclpy.node import Node
from .subscriber import Subscriber


class Streamer:
    def __init__(self, node: Node, client: Client, socket_event: str, socket_callback: Any = None, subscriber: Subscriber = None, function_callback: Any = None):
        self.node = node
        self.client = client
        self.socket_event = socket_event
        self.socket_callback = socket_callback
        self.function_callback = function_callback
        self.subscriber = subscriber if isinstance(
            subscriber, Subscriber) else None

        self._set_strategies()

    def emit_event(self, msg: Any) -> None:
        try:
            json_msg = json.dumps(self._message_to_dictionary(msg))
            self.node.get_logger().info(
                f"Emitting data on {self.socket_event}: {json_msg}")
            self.client.emit(self.socket_event, json_msg)
            self.node.get_logger().info(
                f"Receiving data from topic {self.subscriber.name} and emitting on {self.socket_event}")
            return json_msg
        except Exception as e:
            self.node.get_logger().info(f"Error transforming data: {e}")

    def _message_to_dictionary(self, msg: Any):
        if hasattr(msg, '__slots__'):
            return {slot: self._attribute_strategy(getattr(msg, slot)) for slot in msg.__slots__}
        elif isinstance(msg, float):
            return {"data": msg}
        else:
            self.node.get_logger().info(
                f"Message type {type(msg)} not supported.")

    def _attribute_strategy(self, attribute_value):
        if isinstance(attribute_value, str):
            return attribute_value
        elif hasattr(attribute_value, "__slots__"):
            return self._message_to_dictionary(attribute_value)
        else:
            return attribute_value

    def _set_strategies(self) -> None:
        if self.subscriber is not None:
            self.subscriber.create_sub(self.function_callback)
            self.node.get_logger().info(
                f"Subscriber {self.subscriber.name} created.")
        else:
            self.node.get_logger().info("No subscriber was found on this streamer.")

        if self.socket_callback is not None:
            self.client.on(self.socket_event, self.socket_callback)
            self.node.get_logger().info(
                f"Socket event {self.socket_event} created.")
        else:
            self.node.get_logger().info("No publisher was found on this streamer.")
