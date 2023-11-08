#ifndef TURTLECONTROLLER_H
#define TURTLECONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/kill.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <string.h>
#include <memory>
#include <iostream>

class TurtleController : public rclcpp::Node
{
protected:
   rclcpp::Client<turtlesim::srv::Kill>::SharedPtr killClient_;
   rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawnClient_;
   std::vector<std::string> turtleNames_;

   virtual bool spawnTurtle(std::string turtleName, float x, float y);
   virtual bool killTurtle(std::string turtleName);

public:
   TurtleController(std::string nodeName);
};

#endif // TURTLECONTROLLER_H
