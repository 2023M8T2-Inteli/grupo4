import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from websocket_turtlebot3.subscriber import Subscriber
from websocket_turtlebot3.streamer import Streamer
from websocket_turtlebot3.publisher import Publisher
from geometry_msgs.msg import Pose


class ClientWebSocket(Node):
    def __init__(self):
        super().__init__("client_websocket")

        self.sub = Subscriber(
            self, "status", "/status", String)
        
        self.send_to_queue = Publisher(self, "enqueue", "/enqueue", Pose)
        
        # Callback functions
        websocket_listeners = {"add_to_queue": self.add_to_queue}

        self.streamer = Streamer(sub=self.sub, socket_url="http://localhost:3000", socket_listeners=websocket_listeners)

    def add_to_queue(self, data):
        print("Received data: ", data)
        pose = Pose()
        pose.position.x = data["x"]
        pose.position.y = data["y"]
        pose.position.z = 0.0
        self.send_to_queue.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    battery_node = ClientWebSocket()
    rclpy.spin(battery_node)
    battery_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
