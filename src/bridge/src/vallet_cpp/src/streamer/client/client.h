#ifndef CLIENTCLASS_H
#define CLIENTCLASS_H

#include "rclcpp/rclcpp.hpp"
#include <sio_client.h>

class Client : public rclcpp::Node
{
   public:
      Client();
};

#endif // CLIENTCLASS_H


// 10.128.68.115