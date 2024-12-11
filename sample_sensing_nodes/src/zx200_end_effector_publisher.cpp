#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensing_msgs/msg/zx200_end_effector.hpp"

using namespace std::chrono_literals;


class Zx200EndEffectorPublisher : public rclcpp::Node
{
  public:
    Zx200EndEffectorPublisher()
    : Node("zx200_end_effector_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<sensing_msgs::msg::Zx200EndEffector>("/zx200/end_effector", 10);
      timer_ = this->create_wall_timer(1000ms, std::bind(&Zx200EndEffectorPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = sensing_msgs::msg::Zx200EndEffector();
      message.model_name = "zx200";

      message.record_name = "target_excavate_pose"
      // message.record_name = "target_release_pose";
      // To update target excavate position, please set message.record_name = "target_excavate_pose".
      // To update target position, please set message.record_name = "target_release_pose".
      message.x = x;
      message.y = y;
      message.z = z;
      message.theta_w = theta_w;
      y += 0.1;
      RCLCPP_INFO(this->get_logger(), "Sending the Zx200EndEffector.msg to parameter( model_name = %s,record_name : %s) in mongodb", message.model_name.c_str(),message.record_name.c_str());
      RCLCPP_INFO(this->get_logger(), "x : %f", message.x);
      RCLCPP_INFO(this->get_logger(), "y : %f", message.y);
      RCLCPP_INFO(this->get_logger(), "z : %f", message.z);
      RCLCPP_INFO(this->get_logger(), "theta_w : %f", message.theta_w);
      publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensing_msgs::msg::Zx200EndEffector>::SharedPtr publisher_;
    size_t count_;
    float x=0.0;
    float y=0.0;
    float z=0.0;
    float theta_w=0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Zx200EndEffectorPublisher>());
  rclcpp::shutdown();
  return 0;
}