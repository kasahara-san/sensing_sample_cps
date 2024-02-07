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
      message.model_name = "sample_model";
      message.record_name = "sample_component";
      message.sample_parameter_value = sample_parameter_value;
      RCLCPP_INFO(this->get_logger(), "Sending the value to parameter( model_name = %s, record_name : %s) in mongodb", message.model_name.c_str(),message.record_name.c_str());
      RCLCPP_INFO(this->get_logger(), "sample_parameter_value : %d", message.sample_parameter_value);
      publisher_->publish(message);
      sample_parameter_value += 1;
    }

    int sample_parameter_value = 0;
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