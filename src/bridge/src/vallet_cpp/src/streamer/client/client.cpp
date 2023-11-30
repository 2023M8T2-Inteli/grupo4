#include "client.h"

Client::Client() : rclcpp::Node("STREAMER")
{
   sio::client client;

   client.set_open_listener([&](){
      RCLCPP_INFO(this->get_logger(), "Conectado ao servidor!");
   });

   client.set_close_listener([&](sio::client::close_reason const &reason){
      RCLCPP_INFO(this->get_logger(), "Desconectado do servidor!");
   });

   client.connect("http://localhost:3000");
}