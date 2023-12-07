#ifndef VALLET_CPP_ROBOT_H
#define VALLET_CPP_ROBOT_H

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "slam_toolbox/srv/deserialize_pose_graph.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vallet_msgs/msg/log.hpp"
#include <vector>


enum RobotStatus {
    FREE,
    BUSY,
};

class Robot : public rclcpp::Node {
public:
    typedef geometry_msgs::msg::Pose PoseMsg;
    typedef geometry_msgs::msg::PoseStamped PoseStampedMsg;
    typedef geometry_msgs::msg::PoseWithCovarianceStamped PoseWithCovarianceStampedMsg;
    typedef nav2_msgs::action::NavigateToPose NavigateToPoseAction;
    typedef std_msgs::msg::String StringMsg;
    typedef rclcpp_action::Client<NavigateToPoseAction>::SendGoalOptions Nav2SendGoalOptions;
    typedef action_msgs::msg::GoalStatus GoalStatus;
    typedef rclcpp_action::ClientGoalHandle<NavigateToPoseAction> Nav2GoalHandle;
    typedef vallet_msgs::msg::Log LogMsg;

    typedef rclcpp_action::ClientGoalHandle<NavigateToPoseAction>::WrappedResult Nav2WrappedResult;
    typedef rclcpp_action::Client<NavigateToPoseAction>::SharedPtr Nav2ActionClientPtr;
    typedef rclcpp::Subscription<PoseStampedMsg>::SharedPtr SubscriptionPoseStampedPtr;
    typedef rclcpp::Publisher<PoseWithCovarianceStampedMsg>::SharedPtr PublisherPoseStampedPtr;
    typedef rclcpp::Publisher<StringMsg>::SharedPtr PublisherStringPtr;
    typedef rclcpp::Publisher<LogMsg>::SharedPtr PublisherLogPtr;

    explicit Robot(const std::string &node_name);
    ~Robot() override;

private:
    std::string node_name_;

    Nav2ActionClientPtr action_client_nav2_;
    SubscriptionPoseStampedPtr subscription_;
    PublisherPoseStampedPtr initial_pose_publisher_;
    PublisherLogPtr log_publisher_;
    PublisherStringPtr status_publisher_;

    void publishInitialPose();
    void publishStatus(const RobotStatus &status);

    void subscription_callback(PoseStampedMsg::SharedPtr msg);

    void send_goal(const PoseStampedMsg &goal_pose);

    void send_goal_callback(Nav2GoalHandle::SharedPtr goal_handle);
    void result_send_goal_callback(const Nav2WrappedResult &result);

    LogMsg generate_log_(const std::string &action);
};

#endif//VALLET_CPP_ROBOT_H
