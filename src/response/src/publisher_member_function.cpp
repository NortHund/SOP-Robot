#include <chrono>
#include <memory>
#include <stdio.h>
#include <string.h>

#include "rclcpp/rclcpp.hpp"
#include "vision2_msgs/msg/face.hpp"     // CHANGE

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<vision2_msgs::msg::Face>("topic", 10);    // CHANGE
    timer_ = this->create_wall_timer(
      4000ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = vision2_msgs::msg::Face(); 
    
    if (count_ == 0) {
        message.emotion = "happy";   
        count_++;
    }
    else if (count_ == 1) {
        message.emotion = "sad";
        count_++;
    }
    else if (count_ == 2) {
        message.emotion = "calm";
        count_++;
    }
    else if (count_ == 3) {
        message.emotion = "angry";
        count_++;
    }
    else if (count_ == 4) {
        message.emotion = "surprised";
        count_++;
    }
    else if (count_ == 5) {
        message.emotion = "neutral";
        count_=0;
    }

    RCLCPP_INFO(this->get_logger(), "lähete: '%s'", message.emotion.c_str());    // CHANGE
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<vision2_msgs::msg::Face>::SharedPtr publisher_;         // CHANGE
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

