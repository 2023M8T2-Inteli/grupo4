#include "text_saver.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

TextSaver::TextSaver() : Node("TEXT_SAVER")
{
   this->package_path_ = ament_index_cpp::get_package_share_directory("vallet_cpp");

   RCLCPP_INFO(this->get_logger(), "Creating subscription to /logs topic");

   this->subscription_ = this->create_subscription<vallet_msgs::msg::Log>(
       "/logs", 10, std::bind(&TextSaver::log_callback_, this, std::placeholders::_1));

   RCLCPP_INFO(this->get_logger(), "Initialization COMPLETED!");
}

TextSaver::~TextSaver()
{
   RCLCPP_INFO(this->get_logger(), "Destruction COMPLETED!");
}

void TextSaver::log_callback_(const vallet_msgs::msg::Log::SharedPtr msg)
{
   RCLCPP_INFO(this->get_logger(), "Received log from: %s", msg->node_name.c_str());

   std::ostringstream content_to_save;

   content_to_save << "[" << msg->node_name << "]"
                   << " - " << msg->action << " [" << msg->unix_time << "] "
                   << "\n";

   RCLCPP_INFO(this->get_logger(), "Saving content: %s", content_to_save.str().c_str());

   if (!this->check_path_exists_(std::string("assets")))
   {
      this->create_dirr_(std::string("assets"));
   }

   std::filesystem::path data_file_path = std::filesystem::path(this->package_path_) / std::string("assets/temp_data.txt");

   if (!this->check_path_exists_(std::string("assets/temp_data.txt")))
   {
      this->create_file_(data_file_path);
   }

   this->write_data_(data_file_path, content_to_save.str());

   RCLCPP_INFO(this->get_logger(), "Data wrote in the file: %s", data_file_path.c_str());
}

bool TextSaver::check_path_exists_(const std::string &path)
{
   std::filesystem::path dirr = std::filesystem::path(this->package_path_) / path;

   return std::filesystem::exists(dirr);
}

void TextSaver::create_dirr_(const std::string &folder_name)
{
   std::filesystem::path dirr = std::filesystem::path(this->package_path_) / folder_name;
   std::filesystem::create_directory(dirr);
}

void TextSaver::create_file_(const std::filesystem::path &path)
{
   std::ofstream file_stream(path);
}

void TextSaver::write_data_(const std::filesystem::path &path, const std::string &data)
{
   std::ofstream file_stream(path, std::ios::app);

   if (!file_stream)
   {
      // Trata o erro caso o arquivo não possa ser aberto
      RCLCPP_INFO(this->get_logger(), "It was not possible to open the file: %s", path.c_str());
      return;
   }

   file_stream << data;
}