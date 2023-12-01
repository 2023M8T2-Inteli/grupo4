#include "client/client.h"

int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   sio::client my_client;
   auto client_node = std::make_shared<ClientStreamer>(my_client);
   rclcpp::spin(client_node);
   rclcpp::shutdown();
   return 0;
}