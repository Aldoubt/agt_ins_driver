#include "agt_asensing_driver/msg/ins_status.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

namespace agt_asensing_driver
{
using msg::INSStatus;

class RTKIndicatorNode final : public rclcpp::Node
{
public:
  RTKIndicatorNode() : Node("rtk_indicator")
  {
    status_topic_ = declare_parameter<std::string>("status_topic", "/ins/status");
    timeout_sec_ = declare_parameter<double>("timeout_sec", 2.0);
    print_period_sec_ = declare_parameter<double>("print_period_sec", 1.0);

    indicator_ = create_publisher<std_msgs::msg::String>("/ins/rtk_indicator", 10);
    status_sub_ = create_subscription<INSStatus>(
      status_topic_, 10, std::bind(&RTKIndicatorNode::on_status, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(print_period_sec_)),
      std::bind(&RTKIndicatorNode::report, this));

    RCLCPP_INFO(
      get_logger(), "RTK indicator listening on %s, timeout %.1fs",
      status_topic_.c_str(), timeout_sec_);
  }

private:
  void on_status(const INSStatus::SharedPtr msg)
  {
    last_status_ = *msg;
    last_status_time_ = now();
  }

  void report()
  {
    std_msgs::msg::String out;

    if (!last_status_.has_value()) {
      out.data = "NO_DATA";
      indicator_->publish(out);
      RCLCPP_WARN(get_logger(), "\033[31mRTK [NO DATA]\033[0m waiting for %s", status_topic_.c_str());
      return;
    }

    const auto age = (now() - last_status_time_).seconds();
    if (age > timeout_sec_) {
      out.data = "STALE";
      indicator_->publish(out);
      RCLCPP_WARN(
        get_logger(), "\033[31mRTK [STALE]\033[0m no status for %.1fs, last: %s",
        age, status_summary(*last_status_).c_str());
      return;
    }

    const auto & status = *last_status_;
    out.data = status.rtk_fixed ? "FIXED" : "NOT_FIXED";
    indicator_->publish(out);

    if (status.rtk_fixed) {
      RCLCPP_INFO(
        get_logger(), "\033[32mRTK [FIXED]\033[0m %s",
        status_summary(status).c_str());
    } else {
      RCLCPP_WARN(
        get_logger(), "\033[33mRTK [NOT FIXED]\033[0m %s",
        status_summary(status).c_str());
    }
  }

  static std::string status_summary(const INSStatus & status)
  {
    std::ostringstream ss;
    ss << "pos_type=" << static_cast<int>(status.position_type)
       << " sats=" << static_cast<int>(status.num_satellite);
    if (status.position_std_valid) {
      ss << " std_enu=(" << std::fixed << std::setprecision(3) << status.east_std
         << "," << status.north_std << "," << status.up_std << ")";
    } else {
      ss << " std_enu=n/a";
    }
    ss
       << " heading_type=" << static_cast<int>(status.heading_type)
       << " heading_std=" << std::fixed << std::setprecision(3) << status.heading_std;
    return ss.str();
  }

  std::string status_topic_;
  double timeout_sec_{2.0};
  double print_period_sec_{1.0};
  rclcpp::Subscription<INSStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr indicator_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<INSStatus> last_status_;
  rclcpp::Time last_status_time_;
};
}  // namespace agt_asensing_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agt_asensing_driver::RTKIndicatorNode>());
  rclcpp::shutdown();
  return 0;
}
