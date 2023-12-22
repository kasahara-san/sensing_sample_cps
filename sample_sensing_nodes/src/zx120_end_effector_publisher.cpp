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
      message.parameter_id = 1;
      message.x = 100;
      message.y = 200;
      message.z = 300;
      message.qx = 400;
      message.qy = 500;
      message.qz = 600;
      RCLCPP_INFO(this->get_logger(), "Sending the Zx120EndEffector.msg to parameter_id : 1 in mongodb");
      RCLCPP_INFO(this->get_logger(), "x : %d", message.x);
      RCLCPP_INFO(this->get_logger(), "y : %d", message.y);
      RCLCPP_INFO(this->get_logger(), "z : %d", message.z);
      RCLCPP_INFO(this->get_logger(), "qx : %d", message.qx);
      RCLCPP_INFO(this->get_logger(), "qy : %d", message.qy);
      RCLCPP_INFO(this->get_logger(), "qz : %d", message.qz);
      publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensing_msgs::msg::Zx120EndEffector>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Zx120EndEffectorPublisher>());
  rclcpp::shutdown();
  return 0;
}