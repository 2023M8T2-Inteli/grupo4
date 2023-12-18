import rclpy
from rclpy.node import Node
from nav2_simple_commander.action import CancelTask
from .subscriber import Subscriber
from .publisher import Publisher
from .vallet import NavigatorController
from std_msgs.msg import String


class EmergencyStop(Node):
    def __init__(self):
        super().__init__('cancel_task')
        self.cancel = Subscriber(self, 
                                  "cancel", 
                                  "/cancel", 
                                  CancelTask)
        self.robot_status = Publisher(self, 
                                      "status", 
                                      "/status", 
                                      String)
        self.navigator_controller = NavigatorController(self.nav2_simple_commander)

    def emergency_listener(self, msg):
        self.get_logger().info(f"Received: {msg.content} to stop.")
        # Precisa ver ainda como chega a mensagem do socket, mas olhando os outros presumo que seja nessa estrutura
        if msg.content == True:
            NavigatorController.cancel_task()

    def get_result(self):
        result = self.navigator_controller.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')
        
    def set_robot_status(self):
        self.navigator_controller.publish_status("UNAVAILABLE")

def main(args=None):
    rclpy.init()
    node = EmergencyStop() 
    node.emergency_listener()
    node.set_robot_status()
    node.get_result()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
