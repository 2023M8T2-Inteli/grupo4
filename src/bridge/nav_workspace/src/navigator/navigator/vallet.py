import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from math import pi
from std_msgs.msg import String

class Vallet(Node):

    def __init__(self, navigator):
        super().__init__('vallet')
        self.publisher = self.create_publisher(String, 'status', 10)
        self.subscription = self.create_subscription(
            Pose,
            'poses',
            self.listener_callback,
            10
        )
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.navigator = navigator
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        pos_x = msg.position.x
        pos_y = msg.position.y
        rot_z = 0.0  # Adjust as needed
        goal_pose = create_pose_stamped(self.navigator, pos_x, pos_y, rot_z)
        self.navigator.goToPose(goal_pose)

        
    def timer_callback(self):
        if not self.navigator.isTaskComplete():
            busy_msg = String()
            busy_msg.data = "Busy"
            self.publisher.publish(busy_msg)
        else:
            available_msg = String()
            available_msg.data = "Free"
            self.publisher.publish(available_msg)

def create_pose_stamped(navigator, pos_x, pos_y, rot_z):
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, rot_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = pos_x
    pose.pose.position.y = pos_y
    pose.pose.position.z = pos_x
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()  # Initialize the navigator
    navigator_node = Vallet(navigator)
    rclpy.spin(navigator_node)
    navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
