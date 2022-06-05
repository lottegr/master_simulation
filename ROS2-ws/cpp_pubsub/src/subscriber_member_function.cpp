#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "remote_buddy_common/msg/encoder_data.hpp"                                       // CHANGE

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<remote_buddy_common::msg::EncoderData>(    // CHANGE
      "/encoder_data", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const remote_buddy_common::msg::EncoderData & msg) const  // CHANGE
  {
    int left_ = msg.left_encoder;
    // int right_ = msg.right_encoder;
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << left_ << "'");     // CHANGE
  }
  rclcpp::Subscription<remote_buddy_common::msg::EncoderData>::SharedPtr subscription_;  // CHANGE
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}