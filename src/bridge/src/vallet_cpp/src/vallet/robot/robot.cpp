#include "robot.h"

Robot::Robot(const std::string &node_name) : rclcpp::Node(node_name), node_name_(node_name) {
    RCLCPP_INFO(this->get_logger(), "Creating Robot Node ...");
    RCLCPP_INFO(this->get_logger(), "Creating Nav2 client ...");
    this->action_client_nav2_ = rclcpp_action::create_client<NavigateToPoseAction>(this, "navigate_to_pose");

    RCLCPP_INFO(this->get_logger(), "Creating dequeue subscription ...");
    this->subscription_ = this->create_subscription<PoseStampedMsg>("dequeue", 10, std::bind(&Robot::subscription_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Creating initial pose publisher ...");
    this->initial_pose_publisher_ = this->create_publisher<PoseWithCovarianceStampedMsg>("initialpose", 10);

    RCLCPP_INFO(this->get_logger(), "Creating robot status publisher ...");
    this->status_publisher_ = this->create_publisher<StringMsg>("status", 10);

    RCLCPP_INFO(this->get_logger(), "Creating log publisher ...");
    this->log_publisher_ = this->create_publisher<LogMsg>("/logs", 10);

    while (!this->action_client_nav2_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for the navigation action server to be up...");
    }

    this->publishInitialPose();

    this->generate_log_("ROBOT NODE INITIALIZED");
}

Robot::~Robot() {
    RCLCPP_INFO(this->get_logger(), "Destruction COMPLETED!");
}

void Robot::publishInitialPose() {
    PoseWithCovarianceStampedMsg initial_pose = PoseWithCovarianceStampedMsg();
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = this->get_clock()->now();

    initial_pose.pose.pose.position.x = 0.0;
    initial_pose.pose.pose.position.y = 0.0;
    initial_pose.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    initial_pose.pose.pose.orientation = tf2::toMsg(q);

    RCLCPP_INFO(this->get_logger(), "Publishing initial pose...");
    this->initial_pose_publisher_->publish(initial_pose);

    this->publishStatus(RobotStatus::FREE);

    auto log = this->generate_log_("Initial pose sent");

    this->log_publisher_->publish(log);
}

void Robot::publishStatus(const RobotStatus &status) {
    StringMsg status_msg;

    switch (status) {
        case RobotStatus::FREE:
            status_msg.data = "FREE";
            break;
        case RobotStatus::BUSY:
            status_msg.data = "BUSY";
            break;
        default:
            break;
    }

    RCLCPP_INFO(this->get_logger(), "Publishing status: %s", status_msg.data.c_str());
    status_publisher_->publish(status_msg);
    auto log = this->generate_log_("Status: " + status_msg.data + ", published");

    this->log_publisher_->publish(log);
}

void Robot::subscription_callback(const PoseStampedMsg::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received pose: (%f, %f, %f)", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    auto log = this->generate_log_("Received pose: (" + std::to_string(msg->pose.position.x) + ", " + std::to_string(msg->pose.position.y) + ", " + std::to_string(msg->pose.position.z) + ")");
    this->log_publisher_->publish(log);

    this->send_goal(*msg);
}

void Robot::send_goal(const PoseStampedMsg &goal_pose) {
    auto goal_msg = NavigateToPoseAction::Goal();
    goal_msg.pose = goal_pose;

    auto send_goal_options = Nav2SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&Robot::send_goal_callback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&Robot::result_send_goal_callback, this, std::placeholders::_1);

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto log = this->generate_log_("Sending goal to nav2");
    this->log_publisher_->publish(log);

    this->action_client_nav2_->async_send_goal(goal_msg, send_goal_options);
    this->publishStatus(RobotStatus::BUSY);
}

void Robot::send_goal_callback(Nav2GoalHandle::SharedPtr goal_handle) {
    auto status = goal_handle->get_status();

    switch (status) {
        case GoalStatus::STATUS_ACCEPTED: {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            auto log = this->generate_log_("Goal accepted by server, waiting for result");
            this->log_publisher_->publish(log);
            break;
        }
        default: {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
            auto log = this->generate_log_("Goal rejected by server");
            this->log_publisher_->publish(log);
            this->publishStatus(RobotStatus::FREE);
            break;
        }
    }
    //    if (!goal_handle) {
    //        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    //    } else {
    //        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    //    }
}

void Robot::result_send_goal_callback(const Nav2WrappedResult &result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: {
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            auto log = this->generate_log_("Goal succeeded");
            this->log_publisher_->publish(log);
            break;
        }
        case rclcpp_action::ResultCode::ABORTED: {
            RCLCPP_INFO(this->get_logger(), "Goal was aborted");
            auto log = this->generate_log_("Goal was aborted");
            this->log_publisher_->publish(log);
            break;
        }
        case rclcpp_action::ResultCode::CANCELED: {
            RCLCPP_INFO(this->get_logger(), "Goal was canceled");
            auto log = this->generate_log_("Goal was canceled");
            this->log_publisher_->publish(log);
            break;
        }
        default: {
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            auto log = this->generate_log_("Unknown result code");
            this->log_publisher_->publish(log);
            break;
        }
    }

    this->publishStatus(RobotStatus::FREE);
}

Robot::LogMsg Robot::generate_log_(const std::string &action) {
    LogMsg info;

    info.node_name = this->node_name_;
    info.action = action;
    info.unix_time = std::time(nullptr);

    return info;
}