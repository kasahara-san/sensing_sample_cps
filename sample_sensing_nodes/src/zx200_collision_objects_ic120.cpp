#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensing_msgs/msg/zx200_collision_objects_ic120.hpp"

using namespace std::chrono_literals;


class Zx200CollisionObjectIc120Publisher : public rclcpp::Node
{
  public:
    Zx200CollisionObjectIc120Publisher()
    : Node("zx200_collision_objects_ic120_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<sensing_msgs::msg::Zx200CollisionObjectsIc120>("/zx200/collison_objects_ic120", 10);
      timer_ = this->create_wall_timer(1000ms, std::bind(&Zx200CollisionObjectIc120Publisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        auto message = sensing_msgs::msg::Zx200CollisionObjectsIc120();
        message.model_name = "zx200";
        message.record_name = "collision_object_ic120";
        message.x = x;
        message.y = y;
        message.z = z;
        message.qx = qx;
        message.qy = qy;
        message.qz = qz;
        message.qw = qw;
        y += 0.1;
        RCLCPP_INFO(this->get_logger(), "Sending the Zx200EndEffector.msg to parameter( model_name = %s,record_name : %s) in mongodb", message.model_name.c_str(),message.record_name.c_str());
        RCLCPP_INFO(this->get_logger(), "x : %f", message.x);
        RCLCPP_INFO(this->get_logger(), "y : %f", message.y);
        RCLCPP_INFO(this->get_logger(), "z : %f", message.z);
        RCLCPP_INFO(this->get_logger(), "qx : %f", message.qx);
        RCLCPP_INFO(this->get_logger(), "qy : %f", message.qy);
        RCLCPP_INFO(this->get_logger(), "qz : %f", message.qz);
        RCLCPP_INFO(this->get_logger(), "qw : %f", message.qw);
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensing_msgs::msg::Zx200CollisionObjectsIc120>::SharedPtr publisher_;
    size_t count_;
    float x=0.0;
    float y=0.0;
    float z=0.0;
    float qx=0.0;
    float qy=0.0;
    float qz=0.0;
    float qw=0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Zx200CollisionObjectIc120Publisher>());
  rclcpp::shutdown();
  return 0;
}