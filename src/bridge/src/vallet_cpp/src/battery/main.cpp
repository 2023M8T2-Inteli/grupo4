#include "collector/collector.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto client_node = std::make_unique<ClientStreamer>();
    auto collector = std::make_shared<Collector>("battery", std::move(client_node));

    rclcpp::spin(collector);
    
    rclcpp::shutdown();
    return 0;
}