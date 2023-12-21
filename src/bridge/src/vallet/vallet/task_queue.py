import time
import json
import rclpy
from enum import Enum
from queue import Queue
from . import socket_client
from rclpy.node import Node
from .streamer import Streamer
from vallet_msgs.msg import Task
from .publisher import Publisher
from .subscriber import Subscriber
from std_msgs.msg import String, Int8
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion


class RobotStatus(Enum):
    FREE = 0
    BUSY = 1


class WaitingConfirmation(Enum):
    WAITING = 0
    CONFIRMED = 1


class EmergencyStopState(Enum):
    DISABLED = 0
    ACTIONABLE = 1


class TaskQueue(Node):
    def __init__(self):
        super().__init__("task_queue")

        self.status = Subscriber(self,
                                 "status",
                                 "/status",
                                 String)

        self.dequeue = Publisher(self,
                                 "dequeue",
                                 "/dequeue",
                                 Task)

        self.emergency_stop = Publisher(self,
                                        "emergency_stop",
                                        "/emergency_stop",
                                        Int8)

        self.task_feedback_streamer = Streamer(self,
                                               socket_client,
                                               "/task_feedback")

        self.enqueue_streamer = Streamer(self,
                                         socket_client,
                                         "/enqueue",
                                         self.enqueue_callback)

        self.emergency_stop_streamer = Streamer(self,
                                                socket_client,
                                                "/emergency_stop",
                                                self.emergency_stop_callback)

        self.status.create_sub(self.status_callback)
        self.current: Task = None

        self.queue = Queue()
        self.robot_status = RobotStatus.FREE
        self.waiting_confirmation_state = WaitingConfirmation.CONFIRMED
        self.emergency_stop_state = EmergencyStopState.DISABLED

        self.already_sent = False

    def enqueue_callback(self, msg: dict) -> None:
        try:
            json_msg = json.loads(msg)
        except ValueError as e:
            raise ValueError(f"Invalid JSON: {e}")

        if all(key in json_msg for key in ['id', 'type', 'x', 'y']):
            self._handle_task_data(json_msg)
        else:
            raise TypeError(f"Invalid type: {type(json_msg)}, msg: {json_msg}")

    def emergency_stop_callback(self, msg: str) -> None:
        try:
            json_msg = json.loads(msg)
        except ValueError as e:
            raise ValueError(f"Invalid JSON: {e}")

        if 'emergency_stop' in json_msg:
            self._handle_stop_data(json_msg)
        else:
            raise TypeError(f"Invalid type: {type(json_msg)}, msg: {json_msg}")

    def _handle_stop_data(self, json_msg: dict) -> None:
        if json_msg['emergency_stop'] == 1:
            self.emergency_stop.publish(Int8(data=1))
            self.emergency_stop_state = EmergencyStopState.ACTIONABLE
        elif json_msg['emergency_stop'] == 0:
            self.emergency_stop.publish(Int8(data=0))
            self.emergency_stop_state = EmergencyStopState.DISABLED
            self.waiting_confirmation_state = WaitingConfirmation.CONFIRMED

            self.get_logger().info(f"{json_msg}")
        else:
            self.get_logger().info(f"Invalid emergency stop state: {json_msg}")

    def _handle_task_data(self, json_msg: dict) -> None:
        id = json_msg.get('id')
        self.get_logger().info(f"Received task: {json_msg}")
        task_type = json_msg.get('type')
        x = json_msg.get('x') * 10 / 10
        y = json_msg.get('y') * 10 / 10
        if x is not None and y is not None:
            q_x, q_y, q_z, q_w = quaternion_from_euler(x, y, 0.0)
            task = Task(id=id,
                        type=task_type,
                        pose=Pose(position=Point(x=x, y=y, z=0.0),
                                  orientation=Quaternion(x=q_x, y=q_y, z=q_z, w=q_w)))
            self.queue.put(task)
        else:
            self.get_logger().info(f"Invalid task data: {json_msg}")

    def status_callback(self, msg: String) -> None:
        self.robot_status = RobotStatus.FREE if msg.data == "FREE" else RobotStatus.BUSY
        if self.robot_status == RobotStatus.FREE and self.emergency_stop_state == EmergencyStopState.DISABLED and self.waiting_confirmation_state == WaitingConfirmation.CONFIRMED and not self.queue.empty():
            self.already_sent = False
            self.current = self.queue.get()
            time.sleep(1)
            self.dequeue.publish(self.current)
            if self.current.type == "DROP":
                self.waiting_confirmation_state = WaitingConfirmation.WAITING

        elif self.robot_status == RobotStatus.FREE and self.waiting_confirmation_state == WaitingConfirmation.WAITING and self.already_sent == False:
            self.emergency_stop_state = EmergencyStopState.ACTIONABLE
            self.emergency_stop.publish(Int8(data=1))
            self.task_feedback_streamer.emit_event(self.current)
            self.already_sent = True
        elif self.robot_status == RobotStatus.BUSY and self.emergency_stop_state == EmergencyStopState.DISABLED:
            self.get_logger().info("Robot is busy")
        elif self.robot_status == RobotStatus.BUSY and self.emergency_stop_state == EmergencyStopState.ACTIONABLE:
            self.get_logger().info("Robot is busy and emergency stop is actionable")
        elif self.queue.empty() and self.robot_status == RobotStatus.FREE and self.emergency_stop_state == EmergencyStopState.ACTIONABLE:
            self.get_logger().info("Robot is free and emergency stop is actionable and queue is empty")


def main(args=None):
    rclpy.init(args=args)
    task_queue_node = TaskQueue()
    rclpy.spin(task_queue_node)
    task_queue_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
