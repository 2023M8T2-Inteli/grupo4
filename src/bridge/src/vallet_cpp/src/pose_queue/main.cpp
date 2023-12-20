#include "queue/queue.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto client_node = std::make_unique<ClientStreamer>();
    auto pose_queue = std::make_shared<Queue>("pose_queue", std::move(client_node));

    rclcpp::spin(pose_queue);

    //   rclcpp::spin(pose_queue);
    rclcpp::shutdown();
    return 0;
}