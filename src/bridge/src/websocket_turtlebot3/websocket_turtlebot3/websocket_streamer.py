import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from websocket_turtlebot3.subscriber import Subscriber
from websocket_turtlebot3.streamer import Streamer


class ClientWebSocket(Node):
    def __init__(self):
        super().__init__("battery_percentage")

        self.sub = Subscriber(
            self, "heart_beat_state", "/heart_beat", String)
        self.heart = Streamer(sub=self.sub, socket_url="http://localhost:3000")


def main(args=None):
    rclpy.init(args=args)
    battery_node = ClientWebSocket()
    rclpy.spin(battery_node)
    battery_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
