import rclpy
import socketio
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from .streamer import Streamer
from .publisher import Publisher
from .subscriber import Subscriber


class ClientWebSocket(Node):
    def __init__(self):
        super().__init__("client_websocket")

        self.sio = socketio.Client()

        self.sio.connect("http://localhost:3000")

        self.enqueue = Publisher(self, "enqueue", "/enqueue", Pose)

        self.status = Subscriber(self, "status", "/status", String)

        self.streamer = Streamer(self, self.sio, "/navigation", self.status, self.add_to_queue, self.enqueue)

    def add_to_queue(self, data: dict[str, float]):
        self.get_logger().info(f"Received data: {data} and publishing on {self.enqueue.topic_name}")
        pose = Pose()
        pose.position.x = data["x"]
        pose.position.y = data["y"]
        pose.position.z = 0.0
        self.enqueue.publish(pose)
    
    def disconnect(self) -> None:
        self.get_logger().info("Disconnecting from server.")
        self.destroy_node()
        self.sio.disconnect()

    def disconnect(self):
        self.destroy_node()
        self.sio.disconnect()

def main(args=None):
    rclpy.init(args=args)
    client = ClientWebSocket()
    rclpy.spin(client)
    client.disconnect()
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
