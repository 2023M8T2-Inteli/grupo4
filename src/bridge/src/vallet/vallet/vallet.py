import rclpy
from enum import Enum
from rclpy.node import Node
from .publisher import Publisher
from .subscriber import Subscriber
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Header, Int8
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator

class EmergencyStopState(Enum):
    DISABLED = 0
    ACTIONABLE = 1

class NavigatorController(Node):
    def __init__(self, nav2_simple_commander: BasicNavigator):
        super().__init__("navigator_controller")
        self.robot_status = Publisher(self, 
                                      "status", 
                                      "/status", 
                                      String)
        self.nav2_simple_commander = nav2_simple_commander

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.robot_status.publish(msg)

    def go_to_pose(self, pos_x, pos_y, rot_z):
        goal_pose = self._create_pose_stamped(pos_x, pos_y, rot_z)
        self.nav2_simple_commander.goToPose(goal_pose)

    def is_task_complete(self):
        return self.nav2_simple_commander.isTaskComplete()

    def _create_pose_stamped(self, pos_x, pos_y, pos_z):
        q_x, q_y, q_z, q_w = quaternion_from_euler(pos_x, pos_y, pos_z)
        pose = PoseStamped()
        pose.header = Header(frame_id='map', stamp=self.nav2_simple_commander.get_clock().now().to_msg())
        pose.pose = Pose(
            position=Point(x=pos_x, y=pos_y, z=pos_z),
            orientation=Quaternion(x=q_x, y=q_y, z=q_z, w=q_w)
        )
        return pose


class Vallet(Node):
    def __init__(self, nav2_simple_commander: BasicNavigator):
        super().__init__("navigation")
        self.current_pose: Pose = None
        self.pose = Subscriber(self, 
                               "dequeue", 
                               "/dequeue", 
                               Pose)
        self.emergency_stop = Subscriber(self,
                                         "emergency_stop", 
                                         "/emergency_stop", 
                                         Int8)
        self.pose.create_timer(2.0, self.status_controller)
        self.pose.create_sub(self.listener_callback)
        self.emergency_stop_state = EmergencyStopState.DISABLED
        self.emergency_stop.create_sub(self.emergency_stop_callback)
        self.nav2_simple_commander = nav2_simple_commander
        self.navigator_controller = NavigatorController(
            self.nav2_simple_commander)

    def listener_callback(self, msg: Pose):
        if isinstance(msg, Pose):
            pos_x, pos_y, pos_z = msg.position.x, msg.position.y, msg.position.z
            self.navigator_controller.go_to_pose(pos_x, pos_y, pos_z)
            self.current_pose: Pose = msg
        else:
            self.get_logger().error(f"Invalid message type: {type(msg)}")
    
    def emergency_stop_callback(self, msg: Int8):
        if msg.data == 1 and self.emergency_stop_state == EmergencyStopState.DISABLED:
            self.nav2_simple_commander.cancelTask()
            self.emergency_stop_state = EmergencyStopState.ACTIONABLE
        elif msg.data == 0 and self.emergency_stop_state == EmergencyStopState.ACTIONABLE:
            if self.current_pose is not None:
                self.emergency_stop_state = EmergencyStopState.DISABLED
                self.get_logger().info(f"Current pose: {self.current_pose.position.x}, {self.current_pose.position.y}, {self.current_pose.position.z}")
                self.navigator_controller.go_to_pose(self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z)
            else:
                self.get_logger().info("Current pose is None.")
        else:
            self.get_logger().info(f"Invalid emergency stop state: {msg}")
    
    def status_controller(self):
        if not self.navigator_controller.is_task_complete():
            self.navigator_controller.publish_status("BUSY")
        else:
            self.navigator_controller.publish_status("FREE")


def main(args=None):
    rclpy.init(args=args)
    nav2_simple_commander = BasicNavigator()
    navigation_node = Vallet(nav2_simple_commander)
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
