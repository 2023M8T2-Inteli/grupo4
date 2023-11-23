import json
import socketio
from typing import Any
from rclpy.node import Node
from websocket_turtlebot3.publisher import Publisher
from websocket_turtlebot3.subscriber import Subscriber


class Streamer:
    def __init__(self, node: Node, client: Any, socket_event: str, pub: Publisher = None,  sub: Subscriber = None):
        self.node = node
        self.sio = client
        self.socket_event = socket_event
        self.latest_received_data = None
        self.pub = pub if pub is Publisher else None
        self.sub = sub if sub is Subscriber else None

        self._set_strategies()

    def emit_event(self, msg: Any) -> None:
        try:
            json_msg = json.dumps(self._message_to_dictionary(msg))
            self.sio.emit(self.socket_event, json_msg)
            self.node.get_logger().info(f"Receiving data from topic {self.sub.topic_name} and emitting on {self.socket_event}")
            return json_msg
        except Exception as e:
            raise ValueError(f"Error transforming data: {e}")

    def _message_to_dictionary(self, msg):
        if hasattr(msg, '__slots__'):
            return {slot: self._message_to_dictionary(getattr(msg, slot)) for slot in msg.__slots__}
        elif isinstance(msg, list):
            return [self._message_to_dictionary(sub_msg) for sub_msg in msg]
        else:
            return msg

    def catch_event(self, data: str) -> None:
        try:
            self.latest_received_data = data
            self.node.get_logger().info(f"Received data: {data} and publishing on {self.pub.topic_name}")
            return
        except Exception as e:
            raise ValueError(f"Error sending data: {e}")

    def _set_strategies(self) -> None:
        if self.sub is not None:
            self.sub.create_sub(self.emit_event)
        else:
            raise Exception("No subscriber was found on this streamer.")

        if self.pub is not None:
            self.sio.on(self.socket_event, self.catch_event)
        else:
            raise Exception("No publisher was found on this streamer.")

    def disconnect(self) -> None:
        self.sio.disconnect()
