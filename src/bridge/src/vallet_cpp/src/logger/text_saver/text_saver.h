#ifndef TEXT_SAVER_H
#define TEXT_SAVER_H

#include "rclcpp/rclcpp.hpp"
#include "vallet_msgs/msg/log.hpp"
#include <filesystem>
#include <fstream>

class TextSaver : public rclcpp::Node
{
public:
   TextSaver();
   ~TextSaver();

private:
   rclcpp::Subscription<vallet_msgs::msg::Log>::SharedPtr subscription_;
   std::string package_path_;

   void log_callback_(const vallet_msgs::msg::Log::SharedPtr msg);
   void create_dirr_(const std::string &path);
   void create_file_(const std::filesystem::path &path);
   void write_data_(const std::filesystem::path &path, const std::string &data);
   bool check_path_exists_(const std::string &folder_name);
};

#endif // TEXT_SAVER_H
