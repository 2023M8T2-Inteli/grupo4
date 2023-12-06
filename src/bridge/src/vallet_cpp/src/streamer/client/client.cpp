#include "client.h"
#include <websocketpp/client.hpp>

ClientStreamer::ClientStreamer() : rclcpp::Node("STREAMER"),
                                   _client(std::make_unique<sio::client>()), _connect_finish(false) {

    this->_client->set_open_listener(std::bind(&ClientStreamer::on_connected_, this));

    this->_client->set_close_listener(std::bind(&ClientStreamer::on_close_, this, std::placeholders::_1));

    this->_client->set_fail_listener(std::bind(&ClientStreamer::on_fail_, this));

    this->_client->connect("http://localhost:3000");

    this->_lock.lock();

    if (!this->_connect_finish) {
        RCLCPP_INFO(this->get_logger(), "Waiting SocketIO conection ...");
        _cond.wait(this->_lock);
    }

    this->_lock.unlock();

    RCLCPP_INFO(this->get_logger(), "Start-up COMPLETED!");

    // Example
    // this->emit_("/battery", std::string("Foi apenas um teste"));

    /* Example
    this->_client.socket()->on("message", sio::socket::event_listener_aux(
                                                  [&](std::string const &name, sio::message::ptr const &data, bool isAck,
                                                      sio::message::list &ack_resp) {
                                                      this->on_message_(name, data, isAck, ack_resp);
                                                  }));
    */
}

ClientStreamer::~ClientStreamer() {
    this->_client->sync_close();
    this->_client->clear_con_listeners();
}

void ClientStreamer::on_connected_() {
    this->_lock.lock();
    this->_cond.notify_all();
    this->_connect_finish = true;
    this->_lock.unlock();
}


void ClientStreamer::on_close_(sio::client::close_reason const &reason) {
    std::string reason_str = reason == sio::client::close_reason_normal ? "closed normally" : "closed with reason";

    RCLCPP_INFO(this->get_logger(), reason_str.c_str());
    exit(0);
}

void ClientStreamer::on_fail_() {
    RCLCPP_INFO(this->get_logger(), "SocketIO conection FAILED!");
    exit(0);
}

void ClientStreamer::emit_(const char *topic, const std::string &message) {
    RCLCPP_INFO(this->get_logger(), "Emitindo evento");
    this->_client->socket()->emit(topic, message);
}

void ClientStreamer::on_message_(const std::string &name, sio::message::ptr const &data, bool isAck,
                                 sio::message::list &ack_resp) {
    RCLCPP_INFO(this->get_logger(), "Received data: %s", data->get_string().c_str());
}

template<typename Func>
void ClientStreamer::on_JSON(const std::string &event, Func callback) {
    this->_client->socket()->on(event, sio::socket::event_listener_aux(
                                               [&](std::string const &name, sio::message::ptr const &data, bool isAck,
                                                   sio::message::list &ack_resp) {
                                                   callback(json::parse(data->get_string()));
                                               }));
};