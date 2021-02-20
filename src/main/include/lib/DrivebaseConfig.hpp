#pragma once
#include <lib/ADIS16470Map.hpp>
#include <lib/TalonSRXMap.hpp>
#include <lib/VictorSPXMap.hpp>
#include <lib/ctreJsonSerde.hpp>

namespace rj {
class DrivebaseConfig
{
public:
  TalonSRXMap driveLeft1;
  VictorSPXMap driveLeft2;
  TalonSRXMap driveRight1;
  VictorSPXMap driveRight2;
  ADIS16470Map imu;
  ctre::phoenix::motorcontrol::can::TalonSRXConfiguration talonConfig;
  ctre::phoenix::motorcontrol::can::VictorSPXConfiguration victorConfig;
  float kSLinear;
  float kVLinear;
  float kALinear;
  float kSAngular;
  float kVAngular;
  float kAAngular;
};

} // namespace rj