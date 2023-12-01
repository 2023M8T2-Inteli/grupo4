#include "text_saver/text_saver.h"

int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto text_saver = std::make_shared<TextSaver>();
   rclcpp::spin(text_saver);
   rclcpp::shutdown();
   return 0;
}