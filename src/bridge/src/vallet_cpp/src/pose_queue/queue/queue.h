#ifndef QUEUE_CLASS_H
#define QUEUE_CLASS_H

#include "../../streamer/client/client.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "vallet_msgs/msg/log.hpp"
#include <mutex>
#include <sstream>
#include <vector>

enum RobotStatus {
    FREE,
    BUSY,
    NOT_INITIALIZED
};

class Queue : public rclcpp::Node {
public:
    explicit Queue(const std::string &node_name, std::unique_ptr<ClientStreamer> sio_client);

    ~Queue();

private:
    std::string node_name_;
    std::mutex queue_mutex_;
    std::vector<geometry_msgs::msg::PoseStamped> queue_;
    RobotStatus robot_status_;
    std::unique_ptr<ClientStreamer> sio_client;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr enqueue_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dequeue_publisher_;
    rclcpp::Publisher<vallet_msgs::msg::Log>::SharedPtr log_publisher_;

    void enqueue_callback_(geometry_msgs::msg::Pose::SharedPtr msg);

    void status_callback_(std_msgs::msg::String::SharedPtr msg);

    void timer_callback_();

    vallet_msgs::msg::Log generate_log_(const std::string &action);

    geometry_msgs::msg::PoseStamped create_pose_stamped_(const float &pos_x, const float &pos_y, const float &rot_z);
};


#endif// QUEUE_CLASS_H