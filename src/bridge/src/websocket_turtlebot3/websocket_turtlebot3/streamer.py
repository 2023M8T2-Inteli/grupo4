from websocket_turtlebot3.subscriber import Subscriber
from std_msgs.msg import String
from typing import Any
import json
import socketio


class Streamer:
    def __init__(self, sub: Subscriber, socket_url: str, socket_listeners: dict = {}) -> None:
        self.subscriber = sub
        self.subscriber.create_sub(self.listener_callback)
        self.sio = socketio.Client()
        # Loop para adicionar os listeners 
        if socket_listeners:
            for key, value in socket_listeners.items():
                self.sio.on(str(key), value)

        self.sio.connect(socket_url)
        

    def listener_callback(self, msg: Any) -> None:
        try:
            json_msg = json.dumps(self.message_to_dictionary(msg))
            self.sio.emit("robot_status", json_msg)
            return
        except:
            raise Exception("Error transforming data...")

    def message_to_dictionary(self, msg):
        if hasattr(msg, '__slots__'):
            return {slot: self.message_to_dictionary(getattr(msg, slot)) for slot in msg.__slots__}
        elif isinstance(msg, list):
            return [self.message_to_dictionary(sub_msg) for sub_msg in msg]
        else:
            return msg

    def __del__(self):
        self.sio.disconnect()
