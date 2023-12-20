#include "collector.h"

Collector::Collector(const std::string &node_name, std::unique_ptr<ClientStreamer> sio_client) : rclcpp::Node(node_name), node_name_(node_name), sio_client_(std::move(sio_client)) {
    this->battery_subscription_ = this->create_subscription<BatteryStateMsg>(
            "/battery_state", 10, std::bind(&Collector::battery_state_callback_, this, std::placeholders::_1));

    this->log_publisher_ = this->create_publisher<LogMsg>("/logs", 10);

    RCLCPP_INFO(this->get_logger(), "Collector node started");
}

Collector::~Collector() {
    RCLCPP_INFO(this->get_logger(), "Destruction COMPLETED!");
}

void Collector::battery_state_callback_(const BatteryStateMsg &msg) {
    auto battery_percentage = (msg.voltage - 11) / 1.6 * 100;

    if (battery_percentage >= 0) {
        RCLCPP_INFO(this->get_logger(), "Battery percentage: %f", battery_percentage);

        json json_data;
        json_data["data"] = battery_percentage;

        this->sio_client_->client->socket()->emit("/battery", json_data.dump());

        auto action = "Battery percentage: " + std::to_string(battery_percentage);

        auto log = this->generate_log_(action);

        this->log_publisher_->publish(log);
    } else {
        RCLCPP_INFO(this->get_logger(), "Battery stop been tracked");

        auto log = this->generate_log_("Battery stop been tracked");

        this->log_publisher_->publish(log);
    }
}

Collector::LogMsg Collector::generate_log_(const std::string &action) {
    LogMsg info;

    info.node_name = this->node_name_;
    info.action = action;
    info.unix_time = std::time(nullptr);

    return info;
}