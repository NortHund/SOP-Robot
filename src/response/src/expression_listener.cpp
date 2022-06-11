#include <memory>

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "vision2_msgs/msg/face.hpp"
//#include "vision2_msgs/msg/faceimage.hpp"
//#include "vision2_msgs/msg/faceimages.hpp"
//#include "vision2_msgs/msg/faces.hpp"
//#include "vision2_msgs/msg/point2.hpp"    // CHANGE
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<vision2_msgs::msg::Face>(          // CHANGE
      "face_details", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const vision2_msgs::msg::Face::SharedPtr msg) const       // CHANGE
  {
    RCLCPP_INFO(this->get_logger(), "Näen ilmeen: '%s'", msg->emotion.c_str());
    if (msg->emotion == "happy") { 
        printf("Terve!\n");
        system("ros2 action send_goal /jaw_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"{ trajectory: { joint_names: [head_jaw_joint], points: [ { positions: [0.1], time_from_start: { sec: 1, nanosec: 0 } } ] } }\" ");
        system("canberra-gtk-play -f /workspace/sounds/hello.ogg");
        system("ros2 action send_goal /jaw_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"{ trajectory: { joint_names: [head_jaw_joint], points: [ { positions: [0.0], time_from_start: { sec: 1, nanosec: 0 } } ] } }\" ");
    }
    if (msg->emotion == "angry") { 
        printf("apua!\n");
        system("ros2 action send_goal /head_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"{ trajectory: { joint_names: [head_pan_joint, head_tilt_right_joint, head_tilt_left_joint, head_tilt_vertical_joint], points: [ { positions: [0.7, 0.0, 0.0, 0.0], time_from_start: { sec: 1, nanosec: 0 } } ] } }\" ");
        system("canberra-gtk-play -f /workspace/sounds/helpme.ogg");
        system("ros2 action send_goal /head_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \"{ trajectory: { joint_names: [head_pan_joint, head_tilt_right_joint, head_tilt_left_joint, head_tilt_vertical_joint], points: [ { positions: [0.0, 0.0, 0.0, 0.0], time_from_start: { sec: 1, nanosec: 0 } } ] } }\" ");
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
