#include "agt_asensing_driver/ros_conversions.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

using agt_asensing_driver::INSData;
using agt_asensing_driver::make_imu_message;
using agt_asensing_driver::make_ins_status;
using agt_asensing_driver::make_nav_sat_fix;

TEST(RosGnssContract, UsesGnssFrameAndEnuCovariance)
{
  INSData d;
  d.ins_status = 3;
  d.latitude = 23.1234567;
  d.longitude = 113.1234567;
  d.altitude = 42.5;
  d.latitude_std = 0.20;   // North
  d.longitude_std = 0.10;  // East
  d.altitude_std = 0.30;   // Up
  d.has_position_std = true;

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 123;
  stamp.nanosec = 456;
  const auto fix = make_nav_sat_fix(d, stamp, "rtk_antenna_link");

  EXPECT_EQ(fix.header.frame_id, "rtk_antenna_link");
  EXPECT_EQ(fix.header.stamp.sec, 123);
  EXPECT_EQ(fix.header.stamp.nanosec, 456u);
  EXPECT_EQ(fix.status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);
  EXPECT_NEAR(fix.position_covariance[0], 0.01, 1e-12);  // East
  EXPECT_NEAR(fix.position_covariance[4], 0.04, 1e-12);  // North
  EXPECT_NEAR(fix.position_covariance[8], 0.09, 1e-12);  // Up
  EXPECT_EQ(
    fix.position_covariance_type,
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN);
}

TEST(RosGnssContract, KeepsCovarianceUnknownUntilPositionStdIsValid)
{
  INSData d;
  d.ins_status = 3;
  d.latitude = 23.0;
  d.longitude = 113.0;
  d.altitude = 10.0;
  d.latitude_std = 0.0;
  d.longitude_std = 0.0;
  d.altitude_std = 0.0;
  d.has_position_std = false;

  const auto fix = make_nav_sat_fix(d, builtin_interfaces::msg::Time{}, "rtk_antenna_link");
  EXPECT_EQ(
    fix.position_covariance_type,
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN);
}

TEST(RosGnssContract, NonFiniteCoordinatesAreNotPublishedAsUsableFix)
{
  INSData d;
  d.ins_status = 3;
  d.latitude = std::numeric_limits<double>::quiet_NaN();
  d.longitude = 113.0;
  d.altitude = 10.0;

  const auto fix = make_nav_sat_fix(d, builtin_interfaces::msg::Time{}, "rtk_antenna_link");
  EXPECT_EQ(fix.status.status, sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX);
}

TEST(RosGnssContract, StatusExposesExplicitQualityAndValidity)
{
  INSData d;
  d.ins_status = 3;
  d.position_type = 4;
  d.heading_type = 5;
  d.num_sv = 18;
  d.longitude_std = 0.03;
  d.latitude_std = 0.04;
  d.altitude_std = 0.08;
  d.yaw_std = 0.01;
  d.gps_week = 2234;
  d.gps_time_ms = 123456;
  d.temperature = 35.5F;
  d.wheel_speed_status = 7;
  d.has_position_status = true;
  d.has_position_std = true;
  d.has_attitude_std = true;
  d.has_gps_week = true;
  d.has_temperature = true;
  d.has_wheel_speed_status = true;

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 456;
  const auto status = make_ins_status(d, stamp, "rtk_antenna_link", {4});

  EXPECT_EQ(status.header.frame_id, "rtk_antenna_link");
  EXPECT_EQ(status.header.stamp.sec, 456);
  EXPECT_TRUE(status.position_status_valid);
  EXPECT_TRUE(status.position_std_valid);
  EXPECT_TRUE(status.attitude_std_valid);
  EXPECT_TRUE(status.gps_week_valid);
  EXPECT_TRUE(status.temperature_valid);
  EXPECT_TRUE(status.wheel_speed_status_valid);
  EXPECT_TRUE(status.rtk_fixed);
  EXPECT_FLOAT_EQ(status.east_std, 0.03F);
  EXPECT_FLOAT_EQ(status.north_std, 0.04F);
  EXPECT_FLOAT_EQ(status.up_std, 0.08F);
  EXPECT_FLOAT_EQ(status.position_std, 0.08F);
  EXPECT_EQ(status.num_satellite, 18u);
}

TEST(RosGnssContract, RtkFixedRequiresValidPersistedSolutionStatus)
{
  INSData d;
  d.position_type = 4;
  d.has_position_status = false;
  const auto status = make_ins_status(
    d, builtin_interfaces::msg::Time{}, "rtk_antenna_link", {4});
  EXPECT_FALSE(status.rtk_fixed);
}

TEST(RosImuContract, RawModePublishesInertialDataWithoutTrustedOrientation)
{
  INSData d;
  d.gyro << 0.1, -0.2, 0.3;
  d.accel << 1.0, 2.0, 9.8;
  d.roll = 0.4;
  d.pitch = -0.1;
  d.yaw = 1.2;

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 321;
  stamp.nanosec = 654;
  const auto imu = make_imu_message(d, stamp, "ins_link", false);

  EXPECT_EQ(imu.header.frame_id, "ins_link");
  EXPECT_EQ(imu.header.stamp.sec, 321);
  EXPECT_EQ(imu.header.stamp.nanosec, 654u);
  EXPECT_DOUBLE_EQ(imu.angular_velocity.x, 0.1);
  EXPECT_DOUBLE_EQ(imu.angular_velocity.y, -0.2);
  EXPECT_DOUBLE_EQ(imu.angular_velocity.z, 0.3);
  EXPECT_DOUBLE_EQ(imu.linear_acceleration.x, 1.0);
  EXPECT_DOUBLE_EQ(imu.linear_acceleration.y, 2.0);
  EXPECT_DOUBLE_EQ(imu.linear_acceleration.z, 9.8);

  // The sensor orientation convention is not trusted as ROS REP-103 until R3
  // physical direction tests pass. -1 means orientation estimate unavailable.
  EXPECT_DOUBLE_EQ(imu.orientation_covariance[0], -1.0);

  // Gyroscope and accelerometer measurements are present; all-zero covariance
  // means their covariance is unknown, not that the measurements are absent.
  for (const auto value : imu.angular_velocity_covariance) EXPECT_DOUBLE_EQ(value, 0.0);
  for (const auto value : imu.linear_acceleration_covariance) EXPECT_DOUBLE_EQ(value, 0.0);
}

TEST(RosImuContract, OptInDeviceOrientationUsesAttitudeStd)
{
  INSData d;
  d.roll = 0.0;
  d.pitch = 0.0;
  d.yaw = M_PI_2;
  d.roll_std = 0.01;
  d.pitch_std = 0.02;
  d.yaw_std = 0.03;
  d.has_attitude_std = true;

  const auto imu = make_imu_message(
    d, builtin_interfaces::msg::Time{}, "ins_link", true);

  const double half_sqrt = std::sqrt(0.5);
  EXPECT_NEAR(imu.orientation.x, 0.0, 1e-12);
  EXPECT_NEAR(imu.orientation.y, 0.0, 1e-12);
  EXPECT_NEAR(imu.orientation.z, half_sqrt, 1e-12);
  EXPECT_NEAR(imu.orientation.w, half_sqrt, 1e-12);
  EXPECT_NEAR(imu.orientation_covariance[0], 0.0001, 1e-12);
  EXPECT_NEAR(imu.orientation_covariance[4], 0.0004, 1e-12);
  EXPECT_NEAR(imu.orientation_covariance[8], 0.0009, 1e-12);
}
