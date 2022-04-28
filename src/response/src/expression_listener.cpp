#include <memory>

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "vision2_msgs/msg/face.hpp"     // CHANGE
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<vision2_msgs::msg::Face>(          // CHANGE
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const vision2_msgs::msg::Face::SharedPtr msg) const       // CHANGE
  {
    RCLCPP_INFO(this->get_logger(), "Näen ilmeen: '%s'", msg->emotion.c_str());
    if (msg->emotion == "happy") { 
        printf("Terve!\n");
        system("canberra-gtk-play -f /workspace/sounds/hello.oga");
    }
    if (msg->emotion == "angry") { 
        printf("apua!\n");
        system("canberra-gtk-play -f /workspace/sounds/help.oga");
    }
  }
  rclcpp::Subscription<vision2_msgs::msg::Face>::SharedPtr subscription_;       // CHANGE
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
