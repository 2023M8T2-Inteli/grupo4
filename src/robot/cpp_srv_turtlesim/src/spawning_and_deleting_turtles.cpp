#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/kill.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <string.h>
#include <memory>
#include <iostream>

using namespace std;

class Turtle : public rclcpp::Node
{

private:
   rclcpp::Client<turtlesim::srv::Kill>::SharedPtr killClient_;
   rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawnClient_;

public:
   Turtle(string turtleName) : rclcpp::Node(turtleName)
   {
      killClient_ = this->create_client<turtlesim::srv::Kill>("kill");
      spawnClient_ = this->create_client<turtlesim::srv::Spawn>("spawn");

      while (!killClient_->wait_for_service(std::chrono::seconds(1)) && !spawnClient_->wait_for_service(std::chrono::seconds(1)))
      {
         cout << "waiting for services..." << endl;
      }

      // se já existe uma tartaruga com esse nome, vamos mata-la
      auto killReq = std::make_shared<turtlesim::srv::Kill::Request>();
      killReq->name = turtleName;
      auto promise = this->killClient_->async_send_request(killReq);

      // Waiting for the of the service call
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), promise);

      // spawnando a tartruga
      auto spawnReq = std::make_shared<turtlesim::srv::Spawn::Request>();
      spawnReq->name = turtleName;
      spawnReq->x = 5.54;
      spawnReq->y = 5.54;
      auto response_spawn = this->spawnClient_->async_send_request(spawnReq);

      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), response_spawn) ==
          rclcpp::FutureReturnCode::SUCCESS)
      {
         cout << "Turtle spawned" << endl;
      }
      else
      {
         throw std::runtime_error("Failed to spawn turtle");
      }
   }
};
int main(int argc, char **argv)
{
   // Iniciando o pacote ros
   rclcpp::init(argc, argv);

   // Instanciando a classe do no como um smart pointer
   auto turtle = make_shared<Turtle>("turtle1");

   // Finalizando o ros
   rclcpp::shutdown();

   return 0;
}