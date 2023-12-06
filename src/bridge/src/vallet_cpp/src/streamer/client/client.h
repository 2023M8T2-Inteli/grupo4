#ifndef CLIENTCLASS_H
#define CLIENTCLASS_H

#include "rclcpp/rclcpp.hpp"
#include <condition_variable>
#include <mutex>
#include <nlohmann/json.hpp>
#include <sio_client.h>

using json = nlohmann::json;

// TODO: Remove the Node inheritance
class ClientStreamer : public rclcpp::Node {
public:
    typedef std::function<void(const json&)> callback_json;

    explicit ClientStreamer();
    ~ClientStreamer() override;

    void on_JSON(const std::string &event, const callback_json &&callback);
    std::unique_ptr<sio::client> client;

private:

    std::mutex _lock;
    std::condition_variable_any _cond;
    bool _connect_finish;

    void on_connected_();
    void on_close_(sio::client::close_reason const &reason);
    void on_fail_();
    void emit_(const std::string &topic, const std::string &message);
    void on_message_(const std::string &name, sio::message::ptr const &data, bool isAck, sio::message::list &ack_resp);
};

#endif// CLIENTCLASS_H