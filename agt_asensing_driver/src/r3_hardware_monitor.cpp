#include "agt_asensing_driver/msg/ins_status.hpp"
#include "agt_asensing_driver/r3_metrics.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace agt_asensing_driver
{
namespace
{
using SteadyClock = std::chrono::steady_clock;

int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
    static_cast<int64_t>(stamp.nanosec);
}

std::string json_escape(const std::string & input)
{
  std::ostringstream out;
  for (const char c : input) {
    switch (c) {
      case '\\': out << "\\\\"; break;
      case '"': out << "\\\""; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default: out << c; break;
    }
  }
  return out.str();
}

struct TopicTracker
{
  void observe(const builtin_interfaces::msg::Time & stamp)
  {
    const auto now = SteadyClock::now();
    if (count == 0) {
      first_receive = now;
    } else {
      const double gap = std::chrono::duration<double>(now - last_receive).count();
      max_gap_sec = std::max(max_gap_sec, gap);
    }
    last_receive = now;
    ++count;

    const int64_t ns = stamp_ns(stamp);
    if (last_stamp_ns && ns < *last_stamp_ns) ++stamp_regressions;
    last_stamp_ns = ns;
  }

  double rate_hz() const
  {
    if (count < 2) return 0.0;
    const double elapsed = std::chrono::duration<double>(last_receive - first_receive).count();
    return elapsed > 0.0 ? static_cast<double>(count - 1) / elapsed : 0.0;
  }

  std::size_t count{0};
  std::size_t stamp_regressions{0};
  double max_gap_sec{0.0};
  SteadyClock::time_point first_receive{};
  SteadyClock::time_point last_receive{};
  std::optional<int64_t> last_stamp_ns;
};

struct Event
{
  double elapsed_sec{0.0};
  std::string kind;
  std::string detail;
};

std::string stats_json(const R3RunningStats & stats)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(6)
      << "{\"count\":" << stats.count()
      << ",\"mean\":" << stats.mean()
      << ",\"sample_stddev\":" << stats.sample_stddev()
      << ",\"min\":" << stats.min()
      << ",\"max\":" << stats.max() << "}";
  return out.str();
}

std::string stats_markdown(const R3RunningStats & stats)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(4)
      << "n=" << stats.count()
      << ", mean=" << stats.mean()
      << ", std=" << stats.sample_stddev()
      << ", min=" << stats.min()
      << ", max=" << stats.max();
  return out.str();
}
}  // namespace

class R3HardwareMonitor final : public rclcpp::Node
{
public:
  R3HardwareMonitor()
  : Node("r3_hardware_monitor"), session_start_(SteadyClock::now())
  {
    output_dir_ = declare_parameter("output_dir", "/tmp/agt_ins_r3");
    session_label_ = declare_parameter("session_label", "r3_drive");
    snapshot_period_sec_ = declare_parameter("snapshot_period_sec", 5.0);
    initial_static_window_sec_ = declare_parameter("initial_static_window_sec", 60.0);

    std::filesystem::create_directories(output_dir_);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(200)).reliable();
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/ins/navsatfix", qos,
      std::bind(&R3HardwareMonitor::on_fix, this, std::placeholders::_1));
    status_sub_ = create_subscription<msg::INSStatus>(
      "/ins/status", qos,
      std::bind(&R3HardwareMonitor::on_status, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/ins/imu", qos,
      std::bind(&R3HardwareMonitor::on_imu, this, std::placeholders::_1));
    velocity_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/ins/velocity", qos,
      std::bind(&R3HardwareMonitor::on_velocity, this, std::placeholders::_1));
    marker_sub_ = create_subscription<std_msgs::msg::String>(
      "/ins/r3/marker", 20,
      std::bind(&R3HardwareMonitor::on_marker, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(std::max(1.0, snapshot_period_sec_));
    snapshot_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&R3HardwareMonitor::snapshot, this));

    add_event("monitor", "started");
    RCLCPP_INFO(
      get_logger(),
      "R3 monitor writing snapshots to %s; keep vehicle stationary for the first %.0f s after the first usable GNSS fix",
      output_dir_.c_str(), initial_static_window_sec_);
  }

  void finalize()
  {
    add_event("monitor", "stopped");
    write_reports();
  }

private:
  double elapsed_sec() const
  {
    return std::chrono::duration<double>(SteadyClock::now() - session_start_).count();
  }

  void add_event(const std::string & kind, const std::string & detail)
  {
    events_.push_back(Event{elapsed_sec(), kind, detail});
  }

  void on_fix(const sensor_msgs::msg::NavSatFix::SharedPtr message)
  {
    fix_tracker_.observe(message->header.stamp);
    const bool usable =
      message->status.status != sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX &&
      std::isfinite(message->latitude) && std::isfinite(message->longitude) &&
      std::isfinite(message->altitude) &&
      message->latitude >= -90.0 && message->latitude <= 90.0 &&
      message->longitude >= -180.0 && message->longitude <= 180.0;
    if (!usable) return;

    ++usable_fix_count_;
    if (!reference_latitude_) {
      reference_latitude_ = message->latitude;
      reference_longitude_ = message->longitude;
      reference_altitude_ = message->altitude;
      static_window_start_ = SteadyClock::now();
      add_event("gnss", "first_usable_fix");
    }

    const auto en = approx_enu_offset_m(
      message->latitude, message->longitude,
      *reference_latitude_, *reference_longitude_);
    east_offset_m_.add(en.first);
    north_offset_m_.add(en.second);
    altitude_m_.add(message->altitude);

    if (static_window_start_) {
      const double static_elapsed =
        std::chrono::duration<double>(SteadyClock::now() - *static_window_start_).count();
      if (static_elapsed <= initial_static_window_sec_) {
        static_east_m_.add(en.first);
        static_north_m_.add(en.second);
        static_altitude_m_.add(message->altitude - *reference_altitude_);
      } else if (!static_window_complete_) {
        static_window_complete_ = true;
        add_event("static_window", "complete");
        RCLCPP_INFO(get_logger(), "Initial static window complete; controlled driving may begin");
      }
    }
  }

  void on_status(const msg::INSStatus::SharedPtr message)
  {
    status_tracker_.observe(message->header.stamp);

    if (message->position_status_valid) {
      ++valid_solution_status_count_;
      ++position_type_histogram_[message->position_type];
      satellite_count_.add(message->num_satellite);
      if (message->rtk_fixed) ++rtk_fixed_count_;

      if (last_rtk_fixed_ && *last_rtk_fixed_ != message->rtk_fixed) {
        add_event(
          "rtk_fixed_transition",
          std::string(*last_rtk_fixed_ ? "fixed->not_fixed" : "not_fixed->fixed"));
      }
      last_rtk_fixed_ = message->rtk_fixed;

      if (last_position_type_ && *last_position_type_ != message->position_type) {
        add_event(
          "position_type_transition",
          std::to_string(*last_position_type_) + "->" +
          std::to_string(message->position_type));
      }
      last_position_type_ = message->position_type;
    }

    if (message->position_std_valid) {
      east_std_m_.add(message->east_std);
      north_std_m_.add(message->north_std);
      up_std_m_.add(message->up_std);
    }

    if (message->gps_week_valid) {
      const double device_time = device_time_seconds(message->gps_week, message->gps_time_ms);
      if (!first_device_time_sec_) first_device_time_sec_ = device_time;
      if (last_device_time_sec_ && device_time < *last_device_time_sec_) {
        ++device_time_regressions_;
      }
      last_device_time_sec_ = device_time;
      ++device_time_count_;
    }
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    imu_tracker_.observe(message->header.stamp);
    const double gyro_norm = std::sqrt(
      message->angular_velocity.x * message->angular_velocity.x +
      message->angular_velocity.y * message->angular_velocity.y +
      message->angular_velocity.z * message->angular_velocity.z);
    const double accel_norm = std::sqrt(
      message->linear_acceleration.x * message->linear_acceleration.x +
      message->linear_acceleration.y * message->linear_acceleration.y +
      message->linear_acceleration.z * message->linear_acceleration.z);
    gyro_norm_rad_s_.add(gyro_norm);
    accel_norm_m_s2_.add(accel_norm);
    if (message->orientation_covariance[0] >= 0.0) ++orientation_available_count_;
  }

  void on_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr message)
  {
    velocity_tracker_.observe(message->header.stamp);
    const double horizontal_speed = std::hypot(
      message->twist.linear.x, message->twist.linear.y);
    diagnostic_horizontal_speed_m_s_.add(horizontal_speed);
  }

  void on_marker(const std_msgs::msg::String::SharedPtr message)
  {
    add_event("marker", message->data);
    RCLCPP_INFO(get_logger(), "R3 marker: %s", message->data.c_str());
  }

  double rtk_fixed_ratio() const
  {
    return valid_solution_status_count_ == 0 ? 0.0 :
      static_cast<double>(rtk_fixed_count_) /
      static_cast<double>(valid_solution_status_count_);
  }

  void snapshot()
  {
    write_reports();
    RCLCPP_INFO(
      get_logger(),
      "R3 live nav=%.2fHz status=%.2fHz imu=%.2fHz fixed=%.1f%% sats=%.1f E/N/Ustd=%.3f/%.3f/%.3f",
      fix_tracker_.rate_hz(), status_tracker_.rate_hz(), imu_tracker_.rate_hz(),
      rtk_fixed_ratio() * 100.0, satellite_count_.mean(),
      east_std_m_.mean(), north_std_m_.mean(), up_std_m_.mean());
  }

  std::string tracker_json(const TopicTracker & tracker) const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(6)
        << "{\"count\":" << tracker.count
        << ",\"rate_hz\":" << tracker.rate_hz()
        << ",\"max_receive_gap_sec\":" << tracker.max_gap_sec
        << ",\"header_stamp_regressions\":" << tracker.stamp_regressions << "}";
    return out.str();
  }

  std::string make_json() const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << "{\n"
        << "  \"schema_version\": 1,\n"
        << "  \"session_label\": \"" << json_escape(session_label_) << "\",\n"
        << "  \"elapsed_sec\": " << elapsed_sec() << ",\n"
        << "  \"validation_label\": \"EVIDENCE_ONLY\",\n"
        << "  \"freeze_recommendation\": \"NOT_READY_UNTIL_MANUAL_R3_REVIEW\",\n"
        << "  \"topics\": {\n"
        << "    \"navsatfix\": " << tracker_json(fix_tracker_) << ",\n"
        << "    \"status\": " << tracker_json(status_tracker_) << ",\n"
        << "    \"imu\": " << tracker_json(imu_tracker_) << ",\n"
        << "    \"velocity_diagnostic_only\": " << tracker_json(velocity_tracker_) << "\n"
        << "  },\n"
        << "  \"rtk\": {\n"
        << "    \"usable_fix_count\": " << usable_fix_count_ << ",\n"
        << "    \"valid_solution_status_count\": " << valid_solution_status_count_ << ",\n"
        << "    \"rtk_fixed_count\": " << rtk_fixed_count_ << ",\n"
        << "    \"rtk_fixed_ratio\": " << rtk_fixed_ratio() << ",\n"
        << "    \"satellite_count\": " << stats_json(satellite_count_) << ",\n"
        << "    \"east_std_m\": " << stats_json(east_std_m_) << ",\n"
        << "    \"north_std_m\": " << stats_json(north_std_m_) << ",\n"
        << "    \"up_std_m\": " << stats_json(up_std_m_) << ",\n"
        << "    \"position_type_histogram\": {";
    bool first = true;
    for (const auto & [type, count] : position_type_histogram_) {
      if (!first) out << ",";
      first = false;
      out << "\"" << static_cast<unsigned int>(type) << "\":" << count;
    }
    out << "}\n  },\n";

    out << "  \"position\": {\n";
    if (reference_latitude_) {
      out << "    \"reference_latitude\": " << *reference_latitude_ << ",\n"
          << "    \"reference_longitude\": " << *reference_longitude_ << ",\n"
          << "    \"reference_altitude_m\": " << *reference_altitude_ << ",\n";
    } else {
      out << "    \"reference_latitude\": null,\n"
          << "    \"reference_longitude\": null,\n"
          << "    \"reference_altitude_m\": null,\n";
    }
    out << "    \"trajectory_east_offset_m\": " << stats_json(east_offset_m_) << ",\n"
        << "    \"trajectory_north_offset_m\": " << stats_json(north_offset_m_) << ",\n"
        << "    \"altitude_m\": " << stats_json(altitude_m_) << ",\n"
        << "    \"initial_static_window_sec\": " << initial_static_window_sec_ << ",\n"
        << "    \"initial_static_east_m\": " << stats_json(static_east_m_) << ",\n"
        << "    \"initial_static_north_m\": " << stats_json(static_north_m_) << ",\n"
        << "    \"initial_static_altitude_delta_m\": " << stats_json(static_altitude_m_) << "\n"
        << "  },\n";

    out << "  \"imu\": {\n"
        << "    \"gyro_norm_rad_s\": " << stats_json(gyro_norm_rad_s_) << ",\n"
        << "    \"accel_norm_m_s2\": " << stats_json(accel_norm_m_s2_) << ",\n"
        << "    \"orientation_available_count\": " << orientation_available_count_ << "\n"
        << "  },\n"
        << "  \"velocity_diagnostic_only\": {\n"
        << "    \"horizontal_speed_m_s\": " << stats_json(diagnostic_horizontal_speed_m_s_) << "\n"
        << "  },\n"
        << "  \"device_time\": {\n"
        << "    \"valid_count\": " << device_time_count_ << ",\n"
        << "    \"regressions\": " << device_time_regressions_ << ",\n";
    if (first_device_time_sec_ && last_device_time_sec_) {
      out << "    \"elapsed_sec\": " << (*last_device_time_sec_ - *first_device_time_sec_) << "\n";
    } else {
      out << "    \"elapsed_sec\": null\n";
    }
    out << "  },\n"
        << "  \"events\": [\n";
    for (std::size_t i = 0; i < events_.size(); ++i) {
      const auto & event = events_[i];
      out << "    {\"elapsed_sec\":" << event.elapsed_sec
          << ",\"kind\":\"" << json_escape(event.kind)
          << "\",\"detail\":\"" << json_escape(event.detail) << "\"}";
      if (i + 1 < events_.size()) out << ",";
      out << "\n";
    }
    out << "  ],\n"
        << "  \"manual_checks_required\": [\n"
        << "    \"Confirm ASENSING roll/pitch/yaw axes and signs against ROS REP-103 before enabling device orientation in /ins/imu\",\n"
        << "    \"Confirm vendor position_type values including which value means RTK fixed\",\n"
        << "    \"Observe safe RTK degradation and recovery while stationary or under controlled conditions\",\n"
        << "    \"Verify serial reconnect behavior while vehicle is stationary\",\n"
        << "    \"Measure and record base_link to rtk_antenna_link lever arm\"\n"
        << "  ]\n"
        << "}\n";
    return out.str();
  }

  std::string make_markdown() const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "# ASENSING R3 Hardware Validation Snapshot\n\n"
        << "- Session: `" << session_label_ << "`\n"
        << "- Elapsed: " << elapsed_sec() << " s\n"
        << "- Validation label: **EVIDENCE_ONLY**\n"
        << "- Freeze recommendation: **NOT_READY_UNTIL_MANUAL_R3_REVIEW**\n\n"
        << "## Topic health\n\n"
        << "| Topic | Count | Rate Hz | Max receive gap s | Header regressions |\n"
        << "| --- | ---: | ---: | ---: | ---: |\n"
        << "| /ins/navsatfix | " << fix_tracker_.count << " | " << fix_tracker_.rate_hz()
        << " | " << fix_tracker_.max_gap_sec << " | " << fix_tracker_.stamp_regressions << " |\n"
        << "| /ins/status | " << status_tracker_.count << " | " << status_tracker_.rate_hz()
        << " | " << status_tracker_.max_gap_sec << " | " << status_tracker_.stamp_regressions << " |\n"
        << "| /ins/imu | " << imu_tracker_.count << " | " << imu_tracker_.rate_hz()
        << " | " << imu_tracker_.max_gap_sec << " | " << imu_tracker_.stamp_regressions << " |\n"
        << "| /ins/velocity (diagnostic only) | " << velocity_tracker_.count << " | " << velocity_tracker_.rate_hz()
        << " | " << velocity_tracker_.max_gap_sec << " | " << velocity_tracker_.stamp_regressions << " |\n\n"
        << "## RTK quality\n\n"
        << "- Usable fixes: " << usable_fix_count_ << "\n"
        << "- Valid solution status samples: " << valid_solution_status_count_ << "\n"
        << "- RTK fixed samples: " << rtk_fixed_count_ << "\n"
        << "- RTK fixed ratio: " << rtk_fixed_ratio() * 100.0 << "%\n"
        << "- Satellites: " << stats_markdown(satellite_count_) << "\n"
        << "- East std m: " << stats_markdown(east_std_m_) << "\n"
        << "- North std m: " << stats_markdown(north_std_m_) << "\n"
        << "- Up std m: " << stats_markdown(up_std_m_) << "\n\n"
        << "## Initial static window\n\n"
        << "The first " << initial_static_window_sec_
        << " seconds after the first usable GNSS fix are treated as the initial static window.\n\n"
        << "- East offset m: " << stats_markdown(static_east_m_) << "\n"
        << "- North offset m: " << stats_markdown(static_north_m_) << "\n"
        << "- Altitude delta m: " << stats_markdown(static_altitude_m_) << "\n\n"
        << "## IMU\n\n"
        << "- Gyro norm rad/s: " << stats_markdown(gyro_norm_rad_s_) << "\n"
        << "- Acceleration norm m/s^2: " << stats_markdown(accel_norm_m_s2_) << "\n"
        << "- Samples advertising orientation: " << orientation_available_count_ << "\n\n"
        << "## Device time\n\n"
        << "- Valid GPS-week/time samples: " << device_time_count_ << "\n"
        << "- Device-time regressions: " << device_time_regressions_ << "\n";
    if (first_device_time_sec_ && last_device_time_sec_) {
      out << "- Device elapsed time: " << (*last_device_time_sec_ - *first_device_time_sec_) << " s\n";
    }

    out << "\n## Events / markers\n\n";
    for (const auto & event : events_) {
      out << "- " << event.elapsed_sec << " s — **" << event.kind << "**: " << event.detail << "\n";
    }

    out << "\n## Manual checks still required before R3 PASS\n\n"
        << "- [ ] Verify roll/pitch/yaw axis/sign convention against REP-103 before enabling device orientation in `/ins/imu`.\n"
        << "- [ ] Confirm the ASENSING `position_type` enum from vendor evidence, including RTK fixed.\n"
        << "- [ ] Observe controlled RTK degradation and recovery.\n"
        << "- [ ] Verify serial disconnect/reconnect while stationary.\n"
        << "- [ ] Measure `base_link -> rtk_antenna_link` lever arm.\n"
        << "- [ ] Review rosbag and logs before proposing a candidate SHA/tag.\n";
    return out.str();
  }

  void write_atomic(const std::filesystem::path & path, const std::string & content) const
  {
    const auto temp = path.string() + ".tmp";
    {
      std::ofstream stream(temp, std::ios::trunc);
      stream << content;
    }
    std::error_code ec;
    std::filesystem::rename(temp, path, ec);
    if (ec) {
      std::filesystem::remove(path, ec);
      ec.clear();
      std::filesystem::rename(temp, path, ec);
    }
    if (ec) {
      RCLCPP_ERROR(get_logger(), "Failed writing %s: %s", path.c_str(), ec.message().c_str());
    }
  }

  void write_reports() const
  {
    write_atomic(std::filesystem::path(output_dir_) / "report.json", make_json());
    write_atomic(std::filesystem::path(output_dir_) / "report.md", make_markdown());
  }

  std::string output_dir_;
  std::string session_label_;
  double snapshot_period_sec_{5.0};
  double initial_static_window_sec_{60.0};
  SteadyClock::time_point session_start_;

  TopicTracker fix_tracker_;
  TopicTracker status_tracker_;
  TopicTracker imu_tracker_;
  TopicTracker velocity_tracker_;

  std::size_t usable_fix_count_{0};
  std::size_t valid_solution_status_count_{0};
  std::size_t rtk_fixed_count_{0};
  std::size_t orientation_available_count_{0};
  std::size_t device_time_count_{0};
  std::size_t device_time_regressions_{0};

  std::optional<double> reference_latitude_;
  std::optional<double> reference_longitude_;
  std::optional<double> reference_altitude_;
  std::optional<SteadyClock::time_point> static_window_start_;
  bool static_window_complete_{false};

  std::optional<bool> last_rtk_fixed_;
  std::optional<uint8_t> last_position_type_;
  std::optional<double> first_device_time_sec_;
  std::optional<double> last_device_time_sec_;

  std::map<uint8_t, std::size_t> position_type_histogram_;
  std::vector<Event> events_;

  R3RunningStats satellite_count_;
  R3RunningStats east_std_m_;
  R3RunningStats north_std_m_;
  R3RunningStats up_std_m_;
  R3RunningStats east_offset_m_;
  R3RunningStats north_offset_m_;
  R3RunningStats altitude_m_;
  R3RunningStats static_east_m_;
  R3RunningStats static_north_m_;
  R3RunningStats static_altitude_m_;
  R3RunningStats gyro_norm_rad_s_;
  R3RunningStats accel_norm_m_s2_;
  R3RunningStats diagnostic_horizontal_speed_m_s_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<msg::INSStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr marker_sub_;
  rclcpp::TimerBase::SharedPtr snapshot_timer_;
};
}  // namespace agt_asensing_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agt_asensing_driver::R3HardwareMonitor>();
  rclcpp::spin(node);
  node->finalize();
  rclcpp::shutdown();
  return 0;
}
