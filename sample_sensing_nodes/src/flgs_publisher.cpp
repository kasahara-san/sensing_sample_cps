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
    : Node("flgs_publisher")
    {
      publisher_ = this->create_publisher<sensing_msgs::msg::Flgs>("/Flgs", 10);
      timer_ = this->create_wall_timer(1000ms, std::bind(&FlgsPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = sensing_msgs::msg::Flgs();
      message.record_name = "SAMPLE_BLACKBOARD_SHIMIZU";
      message.continue_flg = true;
      // message.keep_val_flgs = {"CONTINUE_FLG"};
      message.sensing_arrival_flg = false;
      message.sensing_check_mound_flg = false;
      message.sensing_loaded_flg = false;
      RCLCPP_INFO(this->get_logger(), "Sending the Flgs.msg to parameter( record_name : %s) in mongodb",message.record_name.c_str());
      RCLCPP_INFO(this->get_logger(), "continue_flg : %s", message.continue_flg ? "true" : "false");
      RCLCPP_INFO(this->get_logger(), "sensing_arrival_flg : %s", message.sensing_arrival_flg ? "true" : "false");
      RCLCPP_INFO(this->get_logger(), "sensing_check_mound_flg : %s", message.sensing_check_mound_flg ? "true" : "false");
      RCLCPP_INFO(this->get_logger(), "sensing_loaded_flg : %s", message.sensing_loaded_flg ? "true" : "false");
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