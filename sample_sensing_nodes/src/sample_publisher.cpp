#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensing_msgs/msg/sample.hpp"

using namespace std::chrono_literals;


class SensingPublisher : public rclcpp::Node
{
  public:
    SensingPublisher()
    : Node("simple_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<sensing_msgs::msg::Sample>("/sample", 10);
      timer_ = this->create_wall_timer(1000ms, std::bind(&SensingPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = sensing_msgs::msg::Sample();
      message.parameter_id = 0;
      message.parameter_value = parameter_value;
      RCLCPP_INFO(this->get_logger(), "Sending the int32 value %d to parameter_id : 0 in mongodb", message.parameter_value);
      publisher_->publish(message);
      parameter_value += 1;
    }

    int parameter_value = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensing_msgs::msg::Sample>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensingPublisher>());
  rclcpp::shutdown();
  return 0;
}