#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensing_msgs/msg/zx120_end_effector.hpp"

using namespace std::chrono_literals;


class Zx120EndEffectorPublisher : public rclcpp::Node
{
  public:
    Zx120EndEffectorPublisher()
    : Node("simple_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<sensing_msgs::msg::Zx120EndEffector>("/zx120/end_effector", 10);
      timer_ = this->create_wall_timer(1000ms, std::bind(&Zx120EndEffectorPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = sensing_msgs::msg::Zx120EndEffector();
      message.model_name = "zx200";
      message.component_name = "end_effector";
      message.position_with_angle = 1;
      message.x = x;
      message.y = y;
      message.z = z;
      message.theta_w = theta_w;
      y += 0.1;
      RCLCPP_INFO(this->get_logger(), "Sending the Zx120EndEffector.msg to parameter( model_name = %s,component_name : %s) in mongodb", message.model_name.c_str(),message.component_name.c_str());
      RCLCPP_INFO(this->get_logger(), "x : %f", message.x);
      RCLCPP_INFO(this->get_logger(), "y : %f", message.y);
      RCLCPP_INFO(this->get_logger(), "z : %f", message.z);
      RCLCPP_INFO(this->get_logger(), "theta_w : %f", message.theta_w);
      publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensing_msgs::msg::Zx120EndEffector>::SharedPtr publisher_;
    size_t count_;
    float x=0.0;
    float y=0.0;
    float z=0.0;
    float theta_w=0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Zx120EndEffectorPublisher>());
  rclcpp::shutdown();
  return 0;
}