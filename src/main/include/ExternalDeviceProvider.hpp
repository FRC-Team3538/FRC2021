#pragma once

#include "rev/CANSparkMax.h"
#include "rev/ColorSensorV3.h"
#include <AHRS.h>
#include <ctre/Phoenix.h>
#include <frc/Compressor.h>
#include <frc/PowerDistributionPanel.h>
#include <adi/ADIS16470_IMU.h>

#include <iostream>

#include "UDPLogger.hpp"
#include "flatbuffers/flatbuffers.h"
#include "lib/Configuration.hpp"
#include "lib/ctreJsonSerde.hpp"
#include "proto/StatusFrame_generated.h"

using namespace std;
using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;
using namespace frc;

constexpr uint32_t kLeft1 = 0;
constexpr uint32_t kLeft2 = 1;
constexpr uint32_t kRight1 = 3;
constexpr uint32_t kRight2 = 4;

class ExternalDeviceProvider
{
private:
  Configuration config;

  flatbuffers::FlatBufferBuilder fbb;

public:
  void PopulateLogBuffer(UDPLogger& fbb);
  // global
  frc::PowerDistributionPanel pdp{};
  frc::Compressor pcm{};

  // drivetrain
  WPI_TalonFX driveLeft1{ kLeft1 };
  WPI_TalonFX driveRight1{ kRight1 };

  WPI_TalonFX driveLeft2{ kLeft2 };
  WPI_TalonFX driveRight2{ kRight2 };

  AHRS navx{ SPI::Port::kMXP, 200 };

  // climber
  WPI_TalonFX motorClimber0{ 12 };
  WPI_VictorSPX motorClimber1{ 13 };

  // color wheel
  VictorSPX motorColorWheel{ 14 };
  rev::ColorSensorV3 m_colorSensor{ frc::I2C::Port::kOnboard };

  // shooter
  WPI_TalonFX flywheel{ 6 };
  WPI_TalonFX flywheelB{ 7 };
  WPI_TalonFX motorIntake{ 8 };

  WPI_VictorSPX motorIndexer{ 9 };
  WPI_TalonFX motorIndexerB{ 15 };
  WPI_VictorSPX motorIndexerC{ 16 };

  rev::CANSparkMax sparkIndexerB{ 15, rev::CANSparkMax::MotorType::kBrushless };
  rev::CANSparkMax sparkIndexerC{ 16, rev::CANSparkMax::MotorType::kBrushless };

  WPI_TalonSRX motorFeeder{ 10 };
  WPI_VictorSPX motorHood{ 11 };

  // doesn't exist
  // frc::ADIS16470_IMU imu{};
};
