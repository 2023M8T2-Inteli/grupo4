import rclpy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf_transformations import quaternion_from_euler
from nav2_simple_commander.robot_navigator import BasicNavigator


def main(args=None):
    rclpy.init(args=args)
    nav2_simple_commander = BasicNavigator()
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, 0.0)
    initial_pose = PoseStamped()
    initial_pose.header = Header(
        frame_id='map', stamp=nav2_simple_commander.get_clock().now().to_msg())
    initial_pose.pose = Pose(
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=q_x, y=q_y, z=q_z, w=q_w)
    )
    nav2_simple_commander.setInitialPose(initial_pose)
    nav2_simple_commander.waitUntilNav2Active()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
