#ifndef CLIENTCLASS_H
#define CLIENTCLASS_H

#include "rclcpp/rclcpp.hpp"
#include <sio_client.h>
#include <mutex>
#include <condition_variable>

class ClientStreamer : public rclcpp::Node
{
public:
   ClientStreamer(sio::client &client);
   ~ClientStreamer();

private:
   sio::client &_client;
   std::mutex _lock;
   std::condition_variable_any _cond;
   bool _connect_finish;

   void _on_connected();
   void _on_close(sio::client::close_reason const &reason);
   void _on_fail();
   void _emit(const char *topic, const std::string &message);
};

#endif // CLIENTCLASS_H

// 10.128.68.115