#pragma once

#include "rev/CANSparkMax.h"
#include "rev/ColorSensorV3.h"
#include <AHRS.h>
#include <ctre/Phoenix.h>
#include <frc/Compressor.h>
#include <frc/PowerDistributionPanel.h>
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
  TalonFX driveLeft1{ kLeft1 };
  TalonFX driveRight1{ kRight1 };

  TalonFX driveLeft2{ kLeft2 };
  TalonFX driveRight2{ kRight2 };

  AHRS navx{ SPI::Port::kMXP, 200 };

  // climber
  TalonFX motorClimber0{ 12 };
  VictorSPX motorClimber1{ 13 };

  // color wheel
  VictorSPX motorColorWheel{ 14 };
  rev::ColorSensorV3 m_colorSensor{ frc::I2C::Port::kOnboard };

  // shooter
  TalonFX flywheel{ 6 };
  TalonFX flywheelB{ 7 };
  TalonFX motorIntake{ 8 };

  VictorSPX motorIndexer{ 9 };
  TalonFX motorIndexerB{ 15 };
  VictorSPX motorIndexerC{ 16 };

  rev::CANSparkMax sparkIndexerB{ 15, rev::CANSparkMax::MotorType::kBrushless };
  rev::CANSparkMax sparkIndexerC{ 16, rev::CANSparkMax::MotorType::kBrushless };

  TalonSRX motorFeeder{ 10 };
  VictorSPX motorHood{ 11 };
};
