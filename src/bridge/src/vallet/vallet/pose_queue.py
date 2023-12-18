import rclpy
import json
from enum import Enum
from typing import Any
from queue import Queue
from rclpy.node import Node
from .streamer import Streamer
from vallet_msgs.msg import Log
from .publisher import Publisher
from .subscriber import Subscriber
from .__init__ import socket_client
from std_msgs.msg import String, Int8
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion


class RobotStatus(Enum):
    FREE = 0
    BUSY = 1

class EmergencyStopState(Enum):
    DISABLED = 0
    ACTIONABLE = 1

class PoseQueue(Node):
    def __init__(self):
        super().__init__("pose_queue")
        self.enqueue = Subscriber(self, 
                                  "enqueue", 
                                  "/enqueue", 
                                  Pose)
        
        self.status = Subscriber(self, 
                                 "status",
                                 "/status",
                                 String)
        
        self.dequeue = Publisher(self,
                                 "dequeue", 
                                 "/dequeue", 
                                 Pose)
        
        self.emergency_stop = Publisher(self,
                                        "emergency_stop", 
                                        "/emergency_stop", 
                                        Int8)
        
        self.logger = Publisher(self, 
                                "log", 
                                "/logs", 
                                Log)
        
        self.streamer = Streamer(self,
                                 socket_client,
                                 "/enqueue",
                                 self.enqueue_controller)
        
        self.enqueue.create_sub(self.enqueue_controller)
        self.status.create_sub(self.status_callback)

        self.queue = Queue()
        self.robot_status = RobotStatus.FREE
        self.emergency_stop_state = EmergencyStopState.DISABLED

    def enqueue_controller(self, msg: dict) -> None:
        try:
            json_msg = json.loads(msg)
        except ValueError as e:
            raise ValueError(f"Invalid JSON: {e}")
        
        if 'stop' in json_msg:
            self._handle_stop_data(json_msg)
        elif 'x' in json_msg and 'y' in json_msg:
            self._handle_pose_data(json_msg)
        else:
            raise TypeError(f"Invalid type: {type(json_msg)}, msg: {json_msg}")

    def _handle_stop_data(self, json_msg: dict) -> None:
        if json_msg['stop'] == 1:
            self.emergency_stop.publish(Int8(data=1))
            self.emergency_stop_state = EmergencyStopState.ACTIONABLE
        elif json_msg['stop'] == 0:
            self.emergency_stop.publish(Int8(data=0))
            self.emergency_stop_state = EmergencyStopState.DISABLED
            self.get_logger().info(f"{json_msg}")
        else:
            self.get_logger().info(f"Invalid emergency stop state: {json_msg}")

    def _handle_pose_data(self, json_msg: dict) -> None:
        x = json_msg.get('x')
        y = json_msg.get('y')
        if x is not None and y is not None and self.emergency_stop_state == EmergencyStopState.DISABLED:
            q_x, q_y, q_z, q_w = quaternion_from_euler(x, y, 0.0)
            pose = Pose(position=Point(x=x, y=y, z=0.0),
                        orientation=Quaternion(x=q_x, y=q_y, z=q_z, w=q_w))

            log_msg = Log(node_name=self.enqueue.topic_name, 
                          action=f'{json_msg}',
                          unix_time=int(self.get_clock().now().to_msg().sec))
            self.logger.publish(log_msg)

            self.queue.put(pose)
        elif self.emergency_stop_state == EmergencyStopState.ACTIONABLE:
            self.get_logger().info("Emergency stop is actionable")
        else:
            self.get_logger().info(f"Invalid pose data: {json_msg}")

    def status_callback(self, msg: String) -> None:
        self.robot_status = RobotStatus.FREE if msg.data == "FREE" else RobotStatus.BUSY
        self.get_logger().info(f"Robot status: {self.emergency_stop_state}")
        if self.robot_status == RobotStatus.FREE and self.emergency_stop_state == EmergencyStopState.DISABLED and not self.queue.empty():
            self.dequeue.publish(self.queue.get())
        elif self.robot_status == RobotStatus.BUSY and self.emergency_stop_state == EmergencyStopState.DISABLED:
            self.get_logger().info("Robot is busy")
        elif self.robot_status == RobotStatus.BUSY and self.emergency_stop_state == EmergencyStopState.ACTIONABLE:
            self.get_logger().info("Robot is busy and emergency stop is actionable")    
        elif self.queue.empty() and self.robot_status == RobotStatus.FREE and self.emergency_stop_state == EmergencyStopState.ACTIONABLE:
            self.get_logger().info("Robot is free and emergency stop is actionable and queue is empty")

         

def main(args=None):
    rclpy.init(args=args)
    pose_queue_node = PoseQueue()
    rclpy.spin(pose_queue_node)
    pose_queue_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
