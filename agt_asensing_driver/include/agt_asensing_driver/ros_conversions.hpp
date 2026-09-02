#pragma once

#include "agt_asensing_driver/ins_data.hpp"
#include "agt_asensing_driver/msg/ins_status.hpp"

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace agt_asensing_driver
{
namespace detail
{
inline bool finite_nonnegative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

inline bool valid_position_std(const INSData & data)
{
  return data.has_position_std &&
    finite_nonnegative(data.longitude_std) &&
    finite_nonnegative(data.latitude_std) &&
    finite_nonnegative(data.altitude_std);
}

inline bool valid_velocity_std(const INSData & data)
{
  return data.has_velocity_std &&
    finite_nonnegative(data.north_velocity_std) &&
    finite_nonnegative(data.east_velocity_std) &&
    finite_nonnegative(data.ground_velocity_std);
}

inline bool valid_attitude_std(const INSData & data)
{
  return data.has_attitude_std &&
    finite_nonnegative(data.roll_std) &&
    finite_nonnegative(data.pitch_std) &&
    finite_nonnegative(data.yaw_std);
}

inline bool valid_coordinates(const INSData & data)
{
  return std::isfinite(data.latitude) &&
    std::isfinite(data.longitude) &&
    std::isfinite(data.altitude) &&
    data.latitude >= -90.0 && data.latitude <= 90.0 &&
    data.longitude >= -180.0 && data.longitude <= 180.0;
}
}  // namespace detail

inline sensor_msgs::msg::NavSatFix make_nav_sat_fix(
  const INSData & data,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & gnss_frame_id)
{
  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp = stamp;
  fix.header.frame_id = gnss_frame_id;
  fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  fix.status.status =
    data.ins_status != 0 && detail::valid_coordinates(data) ?
    sensor_msgs::msg::NavSatStatus::STATUS_FIX :
    sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;

  fix.latitude = data.latitude;
  fix.longitude = data.longitude;
  fix.altitude = data.altitude;
  fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

  if (detail::valid_position_std(data)) {
    // NavSatFix covariance order is ENU: East, North, Up.
    fix.position_covariance[0] = data.longitude_std * data.longitude_std;
    fix.position_covariance[4] = data.latitude_std * data.latitude_std;
    fix.position_covariance[8] = data.altitude_std * data.altitude_std;
    fix.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  }
  return fix;
}

inline msg::INSStatus make_ins_status(
  const INSData & data,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id,
  const std::vector<int64_t> & rtk_fixed_types)
{
  msg::INSStatus status;
  status.header.stamp = stamp;
  status.header.frame_id = frame_id;

  status.ins_status = data.ins_status;
  status.position_type = data.position_type;
  status.heading_type = data.heading_type;
  status.num_satellite = data.num_sv;

  status.position_status_valid = data.has_position_status;
  status.position_std_valid = detail::valid_position_std(data);
  status.velocity_std_valid = detail::valid_velocity_std(data);
  status.attitude_std_valid = detail::valid_attitude_std(data);
  status.gps_week_valid = data.has_gps_week;
  status.temperature_valid = data.has_temperature && std::isfinite(data.temperature);
  status.wheel_speed_status_valid = data.has_wheel_speed_status;

  if (status.position_std_valid) {
    status.east_std = static_cast<float>(data.longitude_std);
    status.north_std = static_cast<float>(data.latitude_std);
    status.up_std = static_cast<float>(data.altitude_std);
    status.position_std = std::max({status.east_std, status.north_std, status.up_std});
  }

  if (status.velocity_std_valid) {
    status.north_velocity_std = static_cast<float>(data.north_velocity_std);
    status.east_velocity_std = static_cast<float>(data.east_velocity_std);
    status.ground_velocity_std = static_cast<float>(data.ground_velocity_std);
  }

  if (status.attitude_std_valid) {
    status.roll_std = static_cast<float>(data.roll_std);
    status.pitch_std = static_cast<float>(data.pitch_std);
    status.yaw_std = static_cast<float>(data.yaw_std);
    status.heading_std = status.yaw_std;
  }

  status.rtk_fixed = status.position_status_valid &&
    std::find(
      rtk_fixed_types.begin(), rtk_fixed_types.end(),
      static_cast<int64_t>(data.position_type)) != rtk_fixed_types.end();

  status.gps_week = data.gps_week;
  status.gps_time_ms = data.gps_time_ms;
  status.temperature = data.temperature;
  status.wheel_speed_status = data.wheel_speed_status;
  return status;
}
}  // namespace agt_asensing_driver
