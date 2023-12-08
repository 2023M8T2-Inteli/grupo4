#ifndef VALLET_CPP_COLLECTOR_H
#define VALLET_CPP_COLLECTOR_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "vallet_msgs/msg/log.hpp"
#include "../../streamer/client/client.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class Collector : public rclcpp::Node {
public:
    typedef sensor_msgs::msg::BatteryState BatteryStateMsg;
    typedef vallet_msgs::msg::Log LogMsg;

    typedef rclcpp::Subscription<BatteryStateMsg>::SharedPtr SubscriptionBatteryState;
    typedef rclcpp::Publisher<LogMsg>::SharedPtr PublisherLog;


    explicit Collector(const std::string &node_name, std::unique_ptr<ClientStreamer> sio_client);

    ~Collector() override;

private:
    std::string node_name_;

    std::unique_ptr<ClientStreamer> sio_client_;

    SubscriptionBatteryState battery_subscription_;

    PublisherLog log_publisher_;

    void battery_state_callback_(const BatteryStateMsg &msg);

    LogMsg generate_log_(const std::string &action);
};

#endif//VALLET_CPP_COLLECTOR_H
