#include "agt_asensing_driver/r3_metrics.hpp"
#include <gtest/gtest.h>
#include <cmath>

using agt_asensing_driver::R3RunningStats;
using agt_asensing_driver::approx_enu_offset_m;
using agt_asensing_driver::device_time_seconds;

TEST(R3Metrics, RunningStatsTracksMeanAndSampleStddev)
{
  R3RunningStats stats;
  stats.add(1.0);
  stats.add(2.0);
  stats.add(3.0);

  EXPECT_EQ(stats.count(), 3u);
  EXPECT_DOUBLE_EQ(stats.mean(), 2.0);
  EXPECT_NEAR(stats.sample_stddev(), 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(stats.min(), 1.0);
  EXPECT_DOUBLE_EQ(stats.max(), 3.0);
}

TEST(R3Metrics, ApproxEnuUsesEastForLongitudeAndNorthForLatitude)
{
  constexpr double ref_lat = 23.0;
  constexpr double ref_lon = 113.0;

  // About one metre north/east around this latitude.
  const double one_m_lat_deg = 1.0 / 111319.49079327358;
  const double one_m_lon_deg = one_m_lat_deg / std::cos(ref_lat * M_PI / 180.0);

  const auto north = approx_enu_offset_m(
    ref_lat + one_m_lat_deg, ref_lon, ref_lat, ref_lon);
  const auto east = approx_enu_offset_m(
    ref_lat, ref_lon + one_m_lon_deg, ref_lat, ref_lon);

  EXPECT_NEAR(north.first, 0.0, 0.02);   // East
  EXPECT_NEAR(north.second, 1.0, 0.02);  // North
  EXPECT_NEAR(east.first, 1.0, 0.02);    // East
  EXPECT_NEAR(east.second, 0.0, 0.02);   // North
}

TEST(R3Metrics, DeviceTimeSecondsRemainsContinuousAcrossGpsWeekBoundary)
{
  const double end_of_week = device_time_seconds(2300, 604799000u);
  const double next_week = device_time_seconds(2301, 1000u);

  EXPECT_NEAR(next_week - end_of_week, 2.0, 1e-9);
}
