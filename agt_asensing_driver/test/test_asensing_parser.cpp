#include "agt_asensing_driver/asensing_parser.hpp"
#include <gtest/gtest.h>
#include <cmath>

using agt_asensing_driver::ASENSINGParser;

namespace
{
void set_u32(std::vector<uint8_t> & frame, std::size_t offset, uint32_t value)
{
  for (int i = 0; i < 4; ++i) frame[offset + i] = static_cast<uint8_t>(value >> (8 * i));
}

void set_i32(std::vector<uint8_t> & frame, std::size_t offset, int32_t value)
{
  set_u32(frame, offset, static_cast<uint32_t>(value));
}

void set_i16(std::vector<uint8_t> & frame, std::size_t offset, int16_t value)
{
  frame[offset] = static_cast<uint8_t>(value);
  frame[offset + 1] = static_cast<uint8_t>(value >> 8);
}

void set_checksum(std::vector<uint8_t> & frame, std::size_t length)
{
  frame[length - 1] = 0;
  for (std::size_t i = 0; i < length - 1; ++i) frame[length - 1] ^= frame[i];
}

std::vector<uint8_t> make_selector_frame(
  uint8_t selector, int16_t data1, int16_t data2, int16_t data3)
{
  std::vector<uint8_t> frame(58, 0);
  frame[0] = 0xbd; frame[1] = 0xdb; frame[2] = 0x0b;
  frame[39] = 3;
  frame[56] = selector;
  set_i16(frame, 46, data1);
  set_i16(frame, 48, data2);
  set_i16(frame, 50, data3);
  set_checksum(frame, 58);
  return frame;
}

std::vector<uint8_t> make_extended_status_frame(uint32_t gps_week)
{
  std::vector<uint8_t> frame(63, 0);
  frame[0] = 0xbd; frame[1] = 0xdb; frame[2] = 0x0b;
  frame[39] = 3;
  frame[56] = 32;
  set_i16(frame, 46, 4);
  set_i16(frame, 48, 12);
  set_i16(frame, 50, 5);
  set_checksum(frame, 58);
  set_u32(frame, 58, gps_week);
  set_checksum(frame, 63);
  return frame;
}
}  // namespace

TEST(ASENSINGParser, ParsesFragmentedValidMainFrame)
{
  std::vector<uint8_t> frame(58, 0);
  frame[0] = 0xbd; frame[1] = 0xdb; frame[2] = 0x0b;
  frame[21] = 0x78; frame[22] = 0x56; frame[23] = 0x34; frame[24] = 0x12;
  frame[25] = 0xf0; frame[26] = 0xde; frame[27] = 0xbc; frame[28] = 0x9a;
  frame[39] = 3;
  set_checksum(frame, 58);
  ASENSINGParser parser;
  EXPECT_TRUE(parser.feed(frame.data(), 20).empty());
  const auto result = parser.feed(frame.data() + 20, frame.size() - 20);
  ASSERT_EQ(result.size(), 1u);
  EXPECT_NEAR(result[0].latitude, 305419896 * 1e-7, 1e-10);
  EXPECT_NEAR(result[0].longitude, -1698898192 * 1e-7, 1e-10);
  EXPECT_EQ(result[0].ins_status, 3);
}

TEST(ASENSINGParser, ParsesNavigationAndGpsMetadataFromExtendedFrame)
{
  std::vector<uint8_t> frame(63, 0);
  frame[0] = 0xbd; frame[1] = 0xdb; frame[2] = 0x0b;
  set_i32(frame, 21, 315123456);  // 31.5123456 degrees
  set_i32(frame, 25, 1212345678);  // 121.2345678 degrees
  set_i32(frame, 29, 123456);     // 123.456 m
  set_i16(frame, 33, 16384);       // 50 m/s north
  set_i16(frame, 35, -8192);       // -25 m/s east
  set_i16(frame, 37, 3277);        // approximately 10 m/s down
  frame[39] = 3;
  set_u32(frame, 52, 4000000);     // 1,000,000 ms after 0.25 ms scaling
  frame[56] = 32;
  set_i16(frame, 46, 4);
  set_i16(frame, 48, 12);
  set_i16(frame, 50, 5);
  set_checksum(frame, 58);
  set_u32(frame, 58, 2234);
  set_checksum(frame, 63);

  const auto result = ASENSINGParser().feed(frame);
  ASSERT_EQ(result.size(), 1u);
  EXPECT_NEAR(result[0].latitude, 31.5123456, 1e-7);
  EXPECT_NEAR(result[0].longitude, 121.2345678, 1e-7);
  EXPECT_NEAR(result[0].altitude, 123.456, 1e-3);
  EXPECT_NEAR(result[0].north_velocity, 50.0, 1e-6);
  EXPECT_NEAR(result[0].east_velocity, -25.0, 1e-6);
  EXPECT_NEAR(result[0].ground_velocity, 3277 * 100.0 / 32768.0, 1e-6);
  EXPECT_EQ(result[0].gps_week, 2234u);
  EXPECT_EQ(result[0].gps_time_ms, 1000000u);
  EXPECT_EQ(result[0].position_type, 4);
  EXPECT_EQ(result[0].num_sv, 12);
  EXPECT_EQ(result[0].heading_type, 5);
}

TEST(ASENSINGParser, PersistsAuxiliaryMetadataAcrossSelectorFrames)
{
  ASENSINGParser parser;

  const auto position_std = parser.feed(make_selector_frame(0, 0, 69, 110));
  ASSERT_EQ(position_std.size(), 1u);
  EXPECT_TRUE(position_std[0].has_position_std);
  EXPECT_FALSE(position_std[0].has_position_status);
  const double expected_latitude_std = std::exp(0.0);
  const double expected_longitude_std = std::exp(0.69);
  const double expected_altitude_std = std::exp(1.10);

  const auto status = parser.feed(make_selector_frame(32, 4, 12, 5));
  ASSERT_EQ(status.size(), 1u);
  EXPECT_TRUE(status[0].has_position_std);
  EXPECT_TRUE(status[0].has_position_status);
  EXPECT_NEAR(status[0].latitude_std, expected_latitude_std, 1e-12);
  EXPECT_NEAR(status[0].longitude_std, expected_longitude_std, 1e-12);
  EXPECT_NEAR(status[0].altitude_std, expected_altitude_std, 1e-12);
  EXPECT_EQ(status[0].position_type, 4);
  EXPECT_EQ(status[0].num_sv, 12);
  EXPECT_EQ(status[0].heading_type, 5);

  const auto velocity_std = parser.feed(make_selector_frame(1, 10, 20, 30));
  ASSERT_EQ(velocity_std.size(), 1u);
  EXPECT_TRUE(velocity_std[0].has_position_std);
  EXPECT_TRUE(velocity_std[0].has_position_status);
  EXPECT_TRUE(velocity_std[0].has_velocity_std);
  EXPECT_EQ(velocity_std[0].position_type, 4);
  EXPECT_NEAR(velocity_std[0].latitude_std, expected_latitude_std, 1e-12);
  EXPECT_NEAR(velocity_std[0].north_velocity_std, std::exp(0.10), 1e-12);

  const auto attitude_std = parser.feed(make_selector_frame(2, 40, 50, 60));
  ASSERT_EQ(attitude_std.size(), 1u);
  EXPECT_TRUE(attitude_std[0].has_position_std);
  EXPECT_TRUE(attitude_std[0].has_position_status);
  EXPECT_TRUE(attitude_std[0].has_velocity_std);
  EXPECT_TRUE(attitude_std[0].has_attitude_std);
  EXPECT_EQ(attitude_std[0].position_type, 4);
}

TEST(ASENSINGParser, PersistsGpsWeekAfterExtendedFrame)
{
  ASENSINGParser parser;
  const auto with_week = parser.feed(make_extended_status_frame(2234));
  ASSERT_EQ(with_week.size(), 1u);
  EXPECT_TRUE(with_week[0].has_gps_week);
  EXPECT_EQ(with_week[0].gps_week, 2234u);

  const auto next = parser.feed(make_selector_frame(0, 0, 0, 0));
  ASSERT_EQ(next.size(), 1u);
  EXPECT_TRUE(next[0].has_gps_week);
  EXPECT_EQ(next[0].gps_week, 2234u);
}

TEST(ASENSINGParser, TracksOptionalMetadataValidity)
{
  ASENSINGParser parser;
  const auto temperature = parser.feed(make_selector_frame(22, 100, 0, 0));
  ASSERT_EQ(temperature.size(), 1u);
  EXPECT_TRUE(temperature[0].has_temperature);
  EXPECT_FALSE(temperature[0].has_wheel_speed_status);

  const auto wheel = parser.feed(make_selector_frame(33, 0, 7, 0));
  ASSERT_EQ(wheel.size(), 1u);
  EXPECT_TRUE(wheel[0].has_temperature);
  EXPECT_TRUE(wheel[0].has_wheel_speed_status);
  EXPECT_EQ(wheel[0].wheel_speed_status, 7u);
}

TEST(ASENSINGParser, RejectsBadChecksum)
{
  std::vector<uint8_t> frame(58, 0); frame[0] = 0xbd; frame[1] = 0xdb; frame[2] = 0x0b;
  frame[57] = 1;
  EXPECT_TRUE(ASENSINGParser().feed(frame).empty());
}
