import json
from typing import Any
from rclpy.node import Node
from .publisher import Publisher
from .subscriber import Subscriber


class Streamer:
    def __init__(self, node: Node, client: Any, socket_event: str, sub: Subscriber = None, socket_callback: Any = None, pub: Publisher = None):
        self.node = node
        self.sio = client
        self.socket_event = socket_event
        self.latest_received_data = None
        self.pub = pub if isinstance(pub, Publisher) else None
        self.sub = sub if isinstance(sub, Subscriber) else None

        self._set_strategies(socket_callback)

    def emit_event(self, msg: Any) -> None:
        try:
            json_msg = json.dumps(self._message_to_dictionary(msg))
            self.sio.emit(self.socket_event, json_msg)
            self.node.get_logger().info(f"Receiving data from topic {self.sub.name} and emitting on {self.socket_event}")
            return json_msg
        except Exception as e:
            raise ValueError(f"Error transforming data: {e}")

    def _message_to_dictionary(self, msg: Any):
        if hasattr(msg, '__slots__'):
            return {slot: self._message_to_dictionary(getattr(msg, slot)) for slot in msg.__slots__}
        elif isinstance(msg, list):
            return [self._message_to_dictionary(sub_msg) for sub_msg in msg]
        else:
            return msg

    def _set_strategies(self, socket_callback) -> None:
        if self.sub is not None:
            self.sub.create_sub(self.emit_event)
            self.node.get_logger().info(f"Subscriber {self.sub.name} created.")
        else:
            self.node.get_logger().info("No subscriber was found on this streamer.")

        if self.pub is not None:
            self.sio.on(self.socket_event, socket_callback)
            self.node.get_logger().info(f"Socket event {self.socket_event} created.")
        else:
            self.node.get_logger().info("No publisher was found on this streamer.")
