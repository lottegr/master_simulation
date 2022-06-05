// Include important C++ header files that provide class
// templates for useful operations.
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
 
// ROS Client Library for C++
// Allows use of the most common elements of ROS 2
#include "rclcpp/rclcpp.hpp"
#include "remote_buddy_common/msg/encoder_data.hpp"
#include "std_msgs/msg/int64.hpp"
 
using std::placeholders::_1;
 
// Create the node class named PublishingSubscriber which inherits the attributes
// and methods of the rclcpp::Node class.
class PublishingSubscriber : public rclcpp::Node
{
  public:
    // Constructor creates a node named publishing_subscriber. 
    // The published message count is initialized to 0.
    PublishingSubscriber()
    : Node("publishing_subscriber")
    {
      subscription_ = this->create_subscription<remote_buddy_common::msg::EncoderData>(
      "/encoder_data", 10, std::bind(&PublishingSubscriber::topic_callback, this, _1));

      publisher_right_ = this->create_publisher<std_msgs::msg::Int64>("right_encoder",10);
      publisher_left_ = this->create_publisher<std_msgs::msg::Int64>("left_encoder",10);

    }
 
  private:
    // Receives the EncderData message that is published over the topic
    void topic_callback(const remote_buddy_common::msg::EncoderData::SharedPtr msg) const
    {
      // Create two new messages of type Int64
      auto msg_right = std_msgs::msg::Int64();
      auto msg_left = std_msgs::msg::Int64();

      // Set our message's data attribute
      msg_right.data = msg->right_encoder;
      msg_left.data = msg->left_encoder;

      // Publish the messages to the topics
      publisher_right_->publish(msg_right);
      publisher_left_->publish(msg_left);
      RCLCPP_INFO_STREAM(msg_right.data);

    }
    // Declare the subscription attribute
    rclcpp::Subscription<remote_buddy_common::msg::EncoderData>::SharedPtr subscription_;
         
    // Declaration of the publisher_ attribute      
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_right_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_left_;
   
};
 
// Node execution starts here
int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  // Start processing data from the node as well as the callbacks
  rclcpp::spin(std::make_shared<PublishingSubscriber>());
 
  // Shutdown the node when finished
  rclcpp::shutdown();
  return 0;
}




// KILDE: https://automaticaddison.com/create-a-publisher-and-subscriber-in-c-ros-2-foxy-fitzroy/#Create_a_Publishing_Subscriber_Node