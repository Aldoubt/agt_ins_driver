#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>

namespace agt_asensing_driver
{

class R3RunningStats
{
public:
  void add(double value)
  {
    if (!std::isfinite(value)) return;
    ++count_;
    const double delta = value - mean_;
    mean_ += delta / static_cast<double>(count_);
    const double delta2 = value - mean_;
    m2_ += delta * delta2;
    min_ = std::min(min_, value);
    max_ = std::max(max_, value);
  }

  std::size_t count() const { return count_; }
  double mean() const { return count_ == 0 ? 0.0 : mean_; }
  double sample_stddev() const
  {
    return count_ < 2 ? 0.0 : std::sqrt(m2_ / static_cast<double>(count_ - 1));
  }
  double min() const { return count_ == 0 ? 0.0 : min_; }
  double max() const { return count_ == 0 ? 0.0 : max_; }

private:
  std::size_t count_{0};
  double mean_{0.0};
  double m2_{0.0};
  double min_{std::numeric_limits<double>::infinity()};
  double max_{-std::numeric_limits<double>::infinity()};
};

inline std::pair<double, double> approx_enu_offset_m(
  double latitude_deg,
  double longitude_deg,
  double reference_latitude_deg,
  double reference_longitude_deg)
{
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  constexpr double kEarthRadiusM = 6378137.0;
  const double reference_latitude_rad = reference_latitude_deg * kDegToRad;
  const double east =
    (longitude_deg - reference_longitude_deg) * kDegToRad *
    std::cos(reference_latitude_rad) * kEarthRadiusM;
  const double north =
    (latitude_deg - reference_latitude_deg) * kDegToRad * kEarthRadiusM;
  return {east, north};
}

inline double device_time_seconds(uint32_t gps_week, uint32_t gps_time_ms)
{
  constexpr double kSecondsPerGpsWeek = 604800.0;
  return static_cast<double>(gps_week) * kSecondsPerGpsWeek +
    static_cast<double>(gps_time_ms) / 1000.0;
}

}  // namespace agt_asensing_driver
