#include "robot/robot.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto robot = std::make_shared<Robot>("robot");

    rclcpp::spin(robot);

    rclcpp::shutdown();
    return 0;
}