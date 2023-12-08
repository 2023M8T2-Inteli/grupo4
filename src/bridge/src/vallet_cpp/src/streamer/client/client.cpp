#include "client.h"
#include <cstdlib>
#include <websocketpp/client.hpp>

ClientStreamer::ClientStreamer() : rclcpp::Node("STREAMER"),
                                   client(std::make_unique<sio::client>()), _connect_finish(false) {

    this->client->set_open_listener(std::bind(&ClientStreamer::on_connected_, this));

    this->client->set_close_listener(std::bind(&ClientStreamer::on_close_, this, std::placeholders::_1));

    this->client->set_fail_listener(std::bind(&ClientStreamer::on_fail_, this));

    auto url = std::getenv("SOCKET_SERVER_URL") ? std::getenv("SOCKET_SERVER_URL") : "http://localhost:3000";

    RCLCPP_INFO(this->get_logger(), "URL: %s", url);
    //    this->client->connect("http://localhost:3000");
    this->client->connect(url);

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
    this->client->sync_close();
    this->client->clear_con_listeners();
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

void ClientStreamer::emit_(const std::string &topic, const std::string &message) {
    RCLCPP_INFO(this->get_logger(), "Emitindo evento");
    this->client->socket()->emit(topic, message);
}

void ClientStreamer::on_message_(const std::string &name, sio::message::ptr const &data, bool isAck,
                                 sio::message::list &ack_resp) {
    RCLCPP_INFO(this->get_logger(), "Received data: %s", data->get_string().c_str());
}

// FIXME: The callback received is a dinging reference
void ClientStreamer::on_JSON(const std::string &event, callback_json const &&callback) {
    this->client->socket()->on(event, sio::socket::event_listener_aux(
                                              [&](std::string const &name, sio::message::ptr const &data, bool isAck,
                                                  sio::message::list &ack_resp) {
                                                  try {
                                                      RCLCPP_INFO(this->get_logger(), "Received data: %s",
                                                                  data->get_string().c_str());
                                                      json parsedData = json::parse(data->get_string());

                                                      if (callback) {
                                                          callback(json::parse(data->get_string()));
                                                      } else {
                                                          RCLCPP_ERROR(this->get_logger(), "Callback is NULL");
                                                      }
                                                  } catch (const json::parse_error &e) {
                                                      RCLCPP_ERROR(this->get_logger(), "JSON Parse Error: %s", e.what());
                                                      return;
                                                  } catch (const std::exception &e) {
                                                      RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
                                                  }
                                              }));
};