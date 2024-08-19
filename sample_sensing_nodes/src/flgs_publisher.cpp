#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensing_msgs/msg/flgs.hpp"

using namespace std::chrono_literals;


class FlgsPublisher : public rclcpp::Node
{
  public:
    FlgsPublisher()
    : Node("simple_publisher")
    {
      publisher_ = this->create_publisher<sensing_msgs::msg::Flgs>("/Flgs", 10);
      timer_ = this->create_wall_timer(1000ms, std::bind(&FlgsPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = sensing_msgs::msg::Flgs();
      message.record_name = "SAMPLE_BLACKBOARD_SIMIZU";
      //   message.loaded_flg = false;
      message.continue_flg = true;
      message.keep_val_flgs = {"LOADED_FLG"};
      RCLCPP_INFO(this->get_logger(), "Sending the Flgs.msg to parameter( record_name : %s) in mongodb",message.record_name.c_str());
      RCLCPP_INFO(this->get_logger(), "loaded_flg : %f", message.loaded_flg);
      RCLCPP_INFO(this->get_logger(), "continue_flg : %f", message.continue_flg);
      publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensing_msgs::msg::Flgs>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlgsPublisher>());
  rclcpp::shutdown();
  return 0;
}