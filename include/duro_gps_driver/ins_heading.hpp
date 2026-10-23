#ifndef DURO_GPS_DRIVER__INS_HEADING_HPP_
#define DURO_GPS_DRIVER__INS_HEADING_HPP_

#include <cmath>
#include <cstdint>

// The INS heading from SBP ORIENT_EULER (0x0221), free of ROS and libsbp so it can be tested alone.
//
// ORIENT_EULER's yaw is NED: microdegrees, 0 = north, clockwise. ROS (REP 103) yaw is ENU: radians, 0 = east,
// counter-clockwise. While the INS aligns, the Duro still sends ORIENT_EULER with yaw 0 and its INS
// navigation mode (flags bits 0-2) 0, which a driver that ignores the flags turns into a constant +90 deg.
namespace duro_gps_driver
{

struct InsHeading
{
  double yaw_enu;      // rad, in (-pi, pi]
  float accuracy_deg;  // the INS's own 1-sigma yaw accuracy
  bool valid;          // INS navigation mode 1 (valid)
};

inline InsHeading ins_heading(int32_t yaw_microdeg, float yaw_accuracy_deg, uint8_t flags)
{
  const double enu = (90.0 - yaw_microdeg * 1e-6) * M_PI / 180.0;
  return {std::atan2(std::sin(enu), std::cos(enu)), yaw_accuracy_deg, (flags & 0x7) == 1};
}

// The heading is usable when the INS reports it valid and within max_accuracy_deg.
inline bool usable(const InsHeading & h, double max_accuracy_deg)
{
  return h.valid && h.accuracy_deg <= max_accuracy_deg;
}

// Yaw variance for sensor_msgs/Imu, from the reported accuracy, floored at 0.5 deg: the Duro reports
// accuracies well under a degree that the heading does not hold to between epochs.
inline double yaw_variance(const InsHeading & h)
{
  const double sigma = std::fmax(h.accuracy_deg, 0.5) * M_PI / 180.0;
  return sigma * sigma;
}

}  // namespace duro_gps_driver

#endif  // DURO_GPS_DRIVER__INS_HEADING_HPP_
