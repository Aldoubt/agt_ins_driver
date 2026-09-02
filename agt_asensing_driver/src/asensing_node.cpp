#include "agt_asensing_driver/asensing_parser.hpp"
#include "agt_asensing_driver/ros_conversions.hpp"
#include "agt_asensing_driver/serial_port.hpp"
#include "agt_asensing_driver/msg/ins_status.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <cstdint>
#include <vector>

using namespace std::chrono_literals;
namespace agt_asensing_driver
{
using msg::INSStatus;
class ASENSINGNode final : public rclcpp::Node
{
public:
  ASENSINGNode() : Node("asensing_driver")
  {
    port_ = declare_parameter("port", "/dev/ttyUSB0");
    baudrate_ = declare_parameter("baudrate", 230400);
    ins_frame_id_ = declare_parameter("ins_frame_id", "ins_link");
    gnss_frame_id_ = declare_parameter("gnss_frame_id", "rtk_antenna_link");
    rtk_fixed_types_ = declare_parameter<std::vector<int64_t>>("rtk_fixed_types", {4});
    fix_ = create_publisher<sensor_msgs::msg::NavSatFix>("/ins/navsatfix", 10);
    pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("/ins/pose", 10);
    velocity_ = create_publisher<geometry_msgs::msg::TwistStamped>("/ins/velocity", 10);
    odom_ = create_publisher<nav_msgs::msg::Odometry>("/ins/odom", 10);
    status_ = create_publisher<INSStatus>("/ins/status", 10);
    timer_ = create_wall_timer(10ms, std::bind(&ASENSINGNode::poll, this));
  }

private:
  void poll()
  {
    if (!serial_.is_open()) {
      if (!serial_.open(port_, baudrate_)) return;
      RCLCPP_INFO(get_logger(), "Opened %s at %d baud", port_.c_str(), baudrate_);
    }
    const auto bytes = serial_.read_available();
    for (const auto & d : parser_.feed(bytes)) publish(d);
  }

  void publish(const INSData & d)
  {
    // R1/R2 deliberately keep ROS receive time as the measurement stamp. GPS
    // week/time remains available in INSStatus but is not converted to UTC here.
    const auto stamp = now().to_msg();

    const auto fix = make_nav_sat_fix(d, stamp, gnss_frame_id_);
    fix_->publish(fix);

    // These three topics are retained for compatibility. Their full navigation
    // semantics are not part of the accepted R2 contract; consumers should use
    // /ins/navsatfix and /ins/status until a later odometry/velocity review.
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = ins_frame_id_;
    tf2::Quaternion q;
    q.setRPY(d.roll, d.pitch, d.yaw);
    pose.pose.orientation = tf2::toMsg(q);
    pose_->publish(pose);

    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = stamp;
    twist.header.frame_id = ins_frame_id_;
    twist.twist.linear.x = d.north_velocity;
    twist.twist.linear.y = d.east_velocity;
    twist.twist.linear.z = d.ground_velocity;
    velocity_->publish(twist);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = ins_frame_id_;
    odom.child_frame_id = ins_frame_id_;
    tf2::Quaternion odom_q;
    odom_q.setRPY(d.roll, d.pitch, d.yaw);
    odom.pose.pose.orientation = tf2::toMsg(odom_q);
    odom.twist.twist.linear = twist.twist.linear;
    odom.pose.covariance[0] = d.latitude_std * d.latitude_std;
    odom.pose.covariance[7] = d.longitude_std * d.longitude_std;
    odom.pose.covariance[14] = d.altitude_std * d.altitude_std;
    odom.pose.covariance[21] = d.roll_std * d.roll_std;
    odom.pose.covariance[28] = d.pitch_std * d.pitch_std;
    odom.pose.covariance[35] = d.yaw_std * d.yaw_std;
    odom.twist.covariance[0] = d.north_velocity_std * d.north_velocity_std;
    odom.twist.covariance[7] = d.east_velocity_std * d.east_velocity_std;
    odom.twist.covariance[14] = d.ground_velocity_std * d.ground_velocity_std;
    odom_->publish(odom);

    const auto status = make_ins_status(d, stamp, gnss_frame_id_, rtk_fixed_types_);
    status_->publish(status);
  }

  std::string port_, ins_frame_id_, gnss_frame_id_;
  int baudrate_{};
  std::vector<int64_t> rtk_fixed_types_;
  SerialPort serial_;
  ASENSINGParser parser_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Publisher<INSStatus>::SharedPtr status_;
};
}  // namespace agt_asensing_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agt_asensing_driver::ASENSINGNode>());
  rclcpp::shutdown();
  return 0;
}
