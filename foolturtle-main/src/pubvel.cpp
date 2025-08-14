// This program publishes randomly-generated velocity
// messages for turtlesim.

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"  // For geometry_msgs::msg::Twist
#include <stdlib.h> // For rand() and RAND_MAX

using namespace std::chrono_literals;
using std::placeholders::_1;

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode()
  : Node("pubvel_node")
  {
    pubvel_ = create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    //to set a publishing frequency of 2Hz 
    timer_ = create_wall_timer(
      500ms, std::bind(&PublisherNode::timer_callback, this));
    // Seed the random number generator.
    srand(time(0));
  }

  void timer_callback()
  {
    // Create and fill in the message.  The other four
    // fields, which are ignored by turtlesim, default to 0.
    msg_.linear.x = double(rand())/double(RAND_MAX);
    msg_.angular.z = 2*double(rand())/double(RAND_MAX) - 1;
    pubvel_->publish(msg_);
    // Send a message to rosout with the details.
    RCLCPP_INFO_STREAM(get_logger(), "Sending random velocity command:"
      << " linear=" << msg_.linear.x
      << " angular=" << msg_.angular.z);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubvel_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PublisherNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}



