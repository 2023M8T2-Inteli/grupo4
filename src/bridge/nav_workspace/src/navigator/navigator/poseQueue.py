import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from queue import Queue

class PoseQueueNode(Node):

    def __init__(self):
        super().__init__('queue')
        self.pose_queue = Queue()
        self.navigator_free = True
        self.create_subscription(Pose, 'chatbot_msgs', self.cli_callback, 10)
        self.create_subscription(String, 'status', self.status_callback, 10)
        self.pose_publisher = self.create_publisher(Pose, 'poses', 10)

    def cli_callback(self, msg):
        self.pose_queue.put(msg)

    def status_callback(self, msg):
        self.navigator_free = True if msg.data == 'Free' else False
        if self.navigator_free and not self.pose_queue.empty():
            next_pose = self.pose_queue.get()
            print(f'Sending new pose: {next_pose}')
            self.pose_publisher.publish(next_pose)

def main(args=None):
    rclpy.init(args=args)
    node = PoseQueueNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
