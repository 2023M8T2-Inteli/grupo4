#include "client/client.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<ClientStreamer>();
    rclcpp::spin(client_node);
    rclcpp::shutdown();
    return 0;
}