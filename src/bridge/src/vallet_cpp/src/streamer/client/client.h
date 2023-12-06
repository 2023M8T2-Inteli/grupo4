#ifndef CLIENTCLASS_H
#define CLIENTCLASS_H

#include "rclcpp/rclcpp.hpp"
#include <condition_variable>
#include <mutex>
#include <nlohmann/json.hpp>
#include <sio_client.h>

using json = nlohmann::json;
typedef std::function<void(json)> callback_json;

class ClientStreamer : public rclcpp::Node {
public:
    explicit ClientStreamer();
    ~ClientStreamer() override;

    void on_JSON(const std::string &event, callback_json callback);

private:
    std::unique_ptr<sio::client> _client;
    std::mutex _lock;
    std::condition_variable_any _cond;
    bool _connect_finish;

    void on_connected_();
    void on_close_(sio::client::close_reason const &reason);
    void on_fail_();
    void emit_(const char *topic, const std::string &message);
    void on_message_(const std::string &name, sio::message::ptr const &data, bool isAck, sio::message::list &ack_resp);
};

#endif// CLIENTCLASS_H