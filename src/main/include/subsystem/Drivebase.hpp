#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Solenoid.h>
#include <ctre/Phoenix.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/PWMTalonSRX.h>
#include <AHRS.h>
#include <iostream>
// #include "rev/CANSparkMax.h"

using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;
using namespace frc;
// using namespace rev;

class Drivebase
{
private:
  // Hardware setup
  enum motors
  {
    L1 = 0,
    L2,
    L3,
    R1,
    R2,
    R3
  };

  // Talon
  WPI_TalonFX motorLeft1{motors::L1};
  WPI_TalonFX motorLeft2{motors::L2};
  WPI_TalonFX motorLeft3{motors::L3};

  WPI_TalonFX motorRight1{motors::R1};
  WPI_TalonFX motorRight2{motors::R2};
  WPI_TalonFX motorRight3{motors::R3};

  Solenoid solenoidShifter{0};

  // Encoder Scale Factor (Inches)/(Pulse)
  //const double kScaleFactor = (1.0 / 4096.0) * 6 * 3.1415;
  const double kScaleFactor = 53.1875 / 52896;

  enum slots
  {
    Forward = 0,
    Turning,
    Slot2,
    Slot3
  };

  double prevError_rotation = 0;
  double prevError_forward = 0;
  double sumError_forward = 0;
  double prevError_rot = 0;
  double sumError_rotation = 0;
  double target;
  double prevError_rel = 0.0;
  double iAcc = 0.0;

  bool oneShotAngle = false;

#define KP_ROTATION (0.013)    // 0.015
#define KI_ROTATION (0.0)  // 0.00002
#define KD_ROTATION (0.0025) // 0.000015

#define KP_FORWARD (0.013)      // 0.01
#define KI_FORWARD (0.00)      // 0.00
#define KD_FORWARD (0.00)      // 0.00

  SendableChooser<std::string> chooseDriveLimit;
  const std::string sLimited = "Normal";
  const std::string sUnlimited = "Override";

public:
  // Default Constructor
  Drivebase();

  bool sensorOverride = false;
  double forwardHeading = 0;
  // Actions
  void Arcade(double forward, double rotate);

  void Stop();
  void SetHighGear();
  void SetLowGear();
  void SetCoast();
  void SetBrake();

  void SensorOverride(bool active);

  void ResetEncoders();
  double GetEncoderPositionLeft();
  double GetEncoderPositionRight();
  double GetEncoderPosition();

  void ResetGyro();
  double GetGyroHeading();
  double GetPitch();
  void GlobalReset();

  void UpdateSmartdash();

  void DriveForward(double distance, double maxoutput = 1.0);
  void TurnAbs(double degrees, double maxoutput = 1.0);
  bool TurnRel(double degrees, double tolerance);
  void SetMaxSpeed();
  

  AHRS navx{SPI::Port::kMXP, 200};
};