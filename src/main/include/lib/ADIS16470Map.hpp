#pragma once
#include <adi/ADIS16470_IMU.h>
#include <frc/SPI.h>
#include <wpi/json.h>

namespace rj {
class ADIS16470Map
{
public:
  frc::ADIS16470_IMU::IMUAxis yaw_axis;
  frc::SPI::Port port;
  frc::ADIS16470CalibrationTime cal_time;
};
} // namespace rj
