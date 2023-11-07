#include "turtleController.h"
#include <algorithm>

TurtleController::TurtleController(std::string nodeName) : rclcpp::Node(nodeName)
{
   killClient_ = this->create_client<turtlesim::srv::Kill>("kill");
   spawnClient_ = this->create_client<turtlesim::srv::Spawn>("spawn");

   while (!killClient_->wait_for_service(std::chrono::seconds(1)) && !spawnClient_->wait_for_service(std::chrono::seconds(1)))
   {
      std::cout << "waiting for services..." << std::endl;
   }

   // se já existe uma tartaruga com esse nome, vamos mata-la
   auto killFirstTurtleReq = std::make_shared<turtlesim::srv::Kill::Request>();
   killFirstTurtleReq->name = "turtle1";
   auto promiseFirstReq = this->killClient_->async_send_request(killFirstTurtleReq);

   // Waiting for the of the service call
   rclcpp::spin_until_future_complete(this->get_node_base_interface(), promiseFirstReq);
};

bool TurtleController::killTurtle(std::string turtleName)
{
   auto turtleIndex = std::find(this->turtleNames_.begin(), this->turtleNames_.end(), turtleName);

   if (turtleIndex == this->turtleNames_.end())
   {
      std::cout << "Turtle doest exist" << std::endl;
      return false;
   }

   auto killReq = std::make_unique<turtlesim::srv::Kill::Request>();
   killReq->name = turtleName;
   auto promise = this->killClient_->async_send_request(std::move(killReq));

   auto res = rclcpp::spin_until_future_complete(this->get_node_base_interface(), promise);

   if (res == rclcpp::FutureReturnCode::SUCCESS)
   {
      this->turtleNames_.erase(turtleIndex);
      return true;
   }
   else
   {
      std::cout << "Failed to kill turtle" << std::endl;
      return false;
   }
};

bool TurtleController::spawnTurtle(const std::string turtleName, float x, float y)
{

   auto turtleIndex = std::find(this->turtleNames_.begin(), this->turtleNames_.end(), turtleName);

   if (turtleIndex != this->turtleNames_.end())
   {
      std::cout << "Turtle already exists" << std::endl;
      return false;
   }

   std::cout << "Spawning turtle " << turtleName << " in the class TurtleController" << std::endl;

   auto spawnReq = std::make_unique<turtlesim::srv::Spawn::Request>();
   spawnReq->name = turtleName;
   spawnReq->x = x;
   spawnReq->y = y;
   spawnReq->theta = 0.0f;
   auto response_spawn = this->spawnClient_->async_send_request(std::move(spawnReq));

   auto res = rclcpp::spin_until_future_complete(this->get_node_base_interface(), response_spawn);

   if (res == rclcpp::FutureReturnCode::SUCCESS)
   {
      this->turtleNames_.push_back(turtleName);

      std::cout << turtleNames_.back() << " created with success!" << std::endl;

      return true;
   }
   else
   {
      std::cout << "Failed to spawn turtle" << std::endl;
      return false;
   }
};