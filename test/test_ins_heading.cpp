#include <gtest/gtest.h>

#include <cmath>

#include "duro_gps_driver/ins_heading.hpp"

using duro_gps_driver::ins_heading;

constexpr double DEG = M_PI / 180.0;

// Values from real ORIENT_EULER frames in a 2024 Duro Inertial log (swift-gnss-20240207-170704.sbp)
TEST(InsHeading, RealValidFrame)
{
  auto h = ins_heading(-150341064, 0.906f, 0x01);
  EXPECT_TRUE(h.valid);
  EXPECT_NEAR(h.yaw_enu, (90.0 + 150.341064 - 360.0) * DEG, 1e-6);
  EXPECT_TRUE(duro_gps_driver::usable(h, 5.0));
}

TEST(InsHeading, RealAligningFrameIsInvalid)
{
  // yaw 0 and mode 0 while the INS aligns: the old driver published this as +90 deg
  auto h = ins_heading(0, 0.0f, 0x00);
  EXPECT_FALSE(h.valid);
  EXPECT_FALSE(duro_gps_driver::usable(h, 5.0));
}

TEST(InsHeading, CompassToEnu)
{
  const double cases[][2] = {{0, 90}, {90, 0}, {180, -90}, {270, 180}, {-45, 135}};
  for (auto & c : cases) {
    auto h = ins_heading(static_cast<int32_t>(c[0] * 1e6), 1.0f, 0x01);
    EXPECT_NEAR(std::cos(h.yaw_enu), std::cos(c[1] * DEG), 1e-9) << c[0];
    EXPECT_NEAR(std::sin(h.yaw_enu), std::sin(c[1] * DEG), 1e-9) << c[0];
  }
}

TEST(InsHeading, OnlyModeOneIsValid)
{
  for (uint8_t mode = 0; mode < 8; ++mode) {
    EXPECT_EQ(ins_heading(0, 1.0f, mode).valid, mode == 1) << int(mode);
  }
  EXPECT_TRUE(ins_heading(0, 1.0f, 0xF9).valid);  // other bits do not matter
}

TEST(InsHeading, PoorAccuracyIsNotUsable)
{
  EXPECT_FALSE(duro_gps_driver::usable(ins_heading(0, 5.1f, 0x01), 5.0));
  EXPECT_TRUE(duro_gps_driver::usable(ins_heading(0, 5.0f, 0x01), 5.0));
}

TEST(InsHeading, VarianceFloor)
{
  EXPECT_NEAR(duro_gps_driver::yaw_variance(ins_heading(0, 0.1f, 0x01)), std::pow(0.5 * DEG, 2), 1e-12);
  EXPECT_NEAR(duro_gps_driver::yaw_variance(ins_heading(0, 2.0f, 0x01)), std::pow(2.0 * DEG, 2), 1e-9);
}
