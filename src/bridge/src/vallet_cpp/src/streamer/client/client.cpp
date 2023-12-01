#include "client.h"
#include <websocketpp/client.hpp>

ClientStreamer::ClientStreamer(sio::client &client) : rclcpp::Node("STREAMER"), _client(client), _connect_finish(false)
{
   this->_client.set_open_listener(std::bind(&ClientStreamer::_on_connected, this));

   this->_client.set_close_listener(std::bind(&ClientStreamer::_on_close, this, std::placeholders::_1));

   this->_client.set_fail_listener(std::bind(&ClientStreamer::_on_fail, this));

   this->_client.connect("http://localhost:3000");

   this->_lock.lock();

   if (!this->_connect_finish) {
      RCLCPP_INFO(this->get_logger(), "Waiting SocketIO conection ...");
      _cond.wait(this->_lock);
   }

   this->_lock.unlock();

   RCLCPP_INFO(this->get_logger(), "Start-up COMPLETED!");

   this->_emit("/battery", std::string("Foi apenas um teste"));
}

ClientStreamer::~ClientStreamer(){
   this->_client.sync_close();
   this->_client.clear_con_listeners();
}

void ClientStreamer::_on_connected() {
   this->_lock.lock();
   this->_cond.notify_all();
   this->_connect_finish = true;
   this->_lock.unlock();
}


void ClientStreamer::_on_close(sio::client::close_reason const &reason){
   RCLCPP_INFO(this->get_logger(), "SocketIO conection CLOSED!");
   exit(0);
}

void ClientStreamer::_on_fail(){
   RCLCPP_INFO(this->get_logger(), "SocketIO conection FAILED!");
   exit(0);
}

void ClientStreamer::_emit(const char *topic, const std::string &message){
   RCLCPP_INFO(this->get_logger(), "Emitindo evento");
   this->_client.socket()->emit(topic, message);
}

