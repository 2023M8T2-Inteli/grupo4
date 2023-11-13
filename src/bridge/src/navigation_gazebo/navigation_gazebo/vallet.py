import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from navigation_gazebo.publisher import Publisher
from navigation_gazebo.subscriber import Subscriber
from tf_transformations import quaternion_from_euler
from nav2_simple_commander.robot_navigator import BasicNavigator


class NavigatorController(Node):
    def __init__(self, navigator: BasicNavigator):
        super().__init__("navigator_controller")
        self.robot_status = Publisher(self, "status", "/status", String)
        self.navigator = navigator

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.robot_status.publish(msg)

    def go_to_pose(self, pos_x, pos_y, rot_z):
        goal_pose = self._create_pose_stamped(pos_x, pos_y, rot_z)
        self.navigator.goToPose(goal_pose)

    def is_task_complete(self):
        return self.navigator.isTaskComplete()

    def _create_pose_stamped(self, pos_x, pos_y, pos_z):
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, pos_z)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = pos_x
        pose.pose.position.y = pos_y
        pose.pose.position.z = pos_z
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        return pose

    def destroy(self):
        self.robot_status.destroy_pub()
        self.destroy_node()


class Vallet(Node):
    def __init__(self, navigator_controller: NavigatorController):
        super().__init__("vallet")
        self.pose = Subscriber(self, "dequeue", "/dequeue", Pose)
        self.pose.create_timer(1.0, self.timer_callback)
        self.pose.create_sub(self.listener_callback)
        self.navigator_controller = navigator_controller

    def listener_callback(self, msg: Pose):
        pos_x = msg.position.x
        pos_y = msg.position.y
        pos_z = msg.position.z
        self.navigator_controller.go_to_pose(pos_x, pos_y, pos_z)

    def timer_callback(self):
        if not self.navigator_controller.is_task_complete():
            self.navigator_controller.publish_status("BUSY")
        else:
            self.navigator_controller.publish_status("FREE")

    def destroy(self):
        self.pose.destroy_sub()
        self.navigator_controller.destroy()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()
    navigator_controller = NavigatorController(navigator)
    vallet_node = Vallet(navigator_controller)
    rclpy.spin(vallet_node)
    vallet_node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
