import rclpy
import socketio
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from .subscriber import Subscriber
from .streamer import Streamer
from .publisher import Publisher


class ClientWebSocket(Node):
    def __init__(self):
        super().__init__("client_websocket")

        self.sio = socketio.Client()

        self.sio.connect("http://10.128.64.39:3000")

        self.enqueue = Publisher(self, "enqueue", "/enqueue", Pose)

        self.status = Subscriber(self, "status", "/status", String)

        self.streamer = Streamer(self, self.sio, "/navigation", self.enqueue, self.status)

    def add_to_queue(self):
        pose = Pose()
        pose.position.x = self.streamer.latest_received_data["x"]
        pose.position.y = self.streamer.latest_received_data["y"]
        pose.position.z = 0.0
        self.enqueue.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    client = ClientWebSocket()
    client.add_to_queue()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
