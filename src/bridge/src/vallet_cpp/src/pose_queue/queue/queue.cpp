#include "queue.h"

Queue::Queue(const std::string &node_name, std::unique_ptr<ClientStreamer> sio_client) : rclcpp::Node(node_name), node_name_(node_name), robot_status_(RobotStatus::NOT_INITIALIZED),
                                                                                         sio_client(std::move(sio_client)) {
    this->queue_ = std::vector<geometry_msgs::msg::PoseStamped>();

    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(300), std::bind(&Queue::timer_callback_, this));

    // Subs
    enqueue_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "enqueue", 10, std::bind(&Queue::enqueue_callback_, this, std::placeholders::_1));
    status_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "status", 10, std::bind(&Queue::status_callback_, this, std::placeholders::_1));

    // Pubs
    dequeue_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("dequeue", 10);
    log_publisher_ = this->create_publisher<vallet_msgs::msg::Log>("log", 10);

    // SIO Events Listeners

    this->sio_client->on_JSON("/enqueue", [&](const json &data) {
        try {
            RCLCPP_INFO(this->get_logger(), "Received pose: (%f, %f)", data.at("x").get<float>(), data.at("y").get<float>());

            geometry_msgs::msg::Pose::SharedPtr pose;

            pose->position.x = data.at("x").get<float>();
            pose->position.y = data.at("y").get<float>();

            this->enqueue_callback_(pose);

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());

            auto info = this->generate_log_("ERROR: " + std::string(e.what()));

            this->log_publisher_->publish(info);
        }
    });

    /*this->sio_client->socket()->on("message", sio::socket::event_listener_aux(
                                                  [&](std::string const &name, sio::message::ptr const &data, bool isAck,
                                                      sio::message::list &ack_resp) {
                                                      this->on_message_(name, data, isAck, ack_resp);
                                                  }));*/
}

Queue::~Queue() {
    RCLCPP_INFO(this->get_logger(), "Destruction COMPLETED!");
}

void Queue::timer_callback_() {

    switch (this->robot_status_) {
        case RobotStatus::FREE:
            if (this->queue_.empty()) {
                RCLCPP_INFO(this->get_logger(), "Robot is free and queue is empty");

                auto info = this->generate_log_("CHECKING QUEUE. ROBOT IS FREE AND QUEUE IS EMPTY");

                this->log_publisher_->publish(info);
                break;
            } else {
                RCLCPP_INFO(this->get_logger(), "Robot is free and queue is not empty, preparing dequeue ...");
                geometry_msgs::msg::PoseStamped pose = this->queue_.front();
                this->queue_.erase(this->queue_.begin());
                this->dequeue_publisher_->publish(pose);
                RCLCPP_INFO(this->get_logger(), "Pose dequeued and sent to the robot");

                std::ostringstream action;

                action << "POSE (" << pose.pose.position.x << ", " << pose.pose.position.y << ", "
                       << pose.pose.position.z
                       << ") sent to the robot";

                auto info = this->generate_log_(action.str());

                this->log_publisher_->publish(info);

                RCLCPP_INFO(this->get_logger(), "Log sent to the logger node");

                this->robot_status_ = RobotStatus::BUSY;

                break;
            }
        case RobotStatus::BUSY:
            RCLCPP_INFO(this->get_logger(), "Robot is busy");
            if (!this->queue_.empty()) {
                RCLCPP_INFO(this->get_logger(), "Robot is busy and queue is empty");

                auto info = this->generate_log_("CHECKING QUEUE. ROBOT IS BUSY AND QUEUE IS EMPTY");

                this->log_publisher_->publish(info);
                break;
            }
            break;

        case RobotStatus::NOT_INITIALIZED:
            RCLCPP_INFO(this->get_logger(), "Robot is not initialized");
            break;
    }
}

void Queue::status_callback_(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received status: %s", msg->data.c_str());
    if (msg->data == "FREE") {
        switch (this->robot_status_) {
            case RobotStatus::FREE: {
                RCLCPP_INFO(this->get_logger(), "Publishing the action");

                auto info = this->generate_log_("MAINTAINING FREE STATUS");

                this->log_publisher_->publish(info);
                break;
            }
            case RobotStatus::BUSY: {
                RCLCPP_INFO(this->get_logger(), "Publishing the action");

                auto info = this->generate_log_("CHANGING FROM BUSY TO FREE STATUS");

                this->log_publisher_->publish(info);
                break;
            }
            case RobotStatus::NOT_INITIALIZED: {
                RCLCPP_INFO(this->get_logger(), "Publishing the action");

                auto info = this->generate_log_("INITIALIZING THE STATUS FOR THE FIRST TIME");

                this->log_publisher_->publish(info);
                break;
            }
        }
        // Changing the status of the robot
        this->robot_status_ = RobotStatus::FREE;
    } else if (msg->data == "BUSY") {
        switch (this->robot_status_) {
            case RobotStatus::FREE: {
                RCLCPP_INFO(this->get_logger(), "Publishing the action");

                auto info = this->generate_log_("CHANGING FROM FREE TO BUSY STATUS");

                this->log_publisher_->publish(info);
                break;
            }
            case RobotStatus::BUSY: {
                RCLCPP_INFO(this->get_logger(), "Publishing the action");

                auto info = this->generate_log_("MAINTAINING BUSY STATUS");

                this->log_publisher_->publish(info);
                break;
            }
            case RobotStatus::NOT_INITIALIZED: {
                RCLCPP_INFO(this->get_logger(), "Publishing the action");

                auto info = this->generate_log_("INITIALIZING THE STATUS FOR THE FIRST TIME");

                this->log_publisher_->publish(info);
                break;
            }
        }
        // Changing the status of the robot
        this->robot_status_ = RobotStatus::BUSY;
    }
}

void Queue::enqueue_callback_(const geometry_msgs::msg::Pose::SharedPtr msg) {
    std::unique_lock<std::mutex> lock(this->queue_mutex_);

    RCLCPP_INFO(this->get_logger(), "Received pose: (%f, %f, %f)", msg->position.x, msg->position.y, msg->position.z);

    geometry_msgs::msg::PoseStamped pose_stamped = this->create_pose_stamped_(msg->position.x, msg->position.y, 0);

    this->queue_.push_back(pose_stamped);

    lock.unlock();

    RCLCPP_INFO(this->get_logger(), "Pose enqueued");

    std::ostringstream action;
    action << "POSE (" << msg->position.x << ", " << msg->position.y << ", " << msg->position.z
           << ") sent to the robot";

    auto info = this->generate_log_(action.str());

    this->log_publisher_->publish(info);
}

vallet_msgs::msg::Log Queue::generate_log_(const std::string &action) {
    vallet_msgs::msg::Log info;

    info.node_name = this->node_name_;
    info.action = action;
    info.unix_time = std::time(nullptr);

    return info;
}

geometry_msgs::msg::PoseStamped
Queue::create_pose_stamped_(const float &pos_x, const float &pos_y, const float &rot_z) {
    tf2::Quaternion q;
    q.setRPY(0, 0, rot_z);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->get_clock()->now();
    pose.pose.position.x = pos_x;
    pose.pose.position.y = pos_y;
    pose.pose.position.z = 0;
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    return pose;
}