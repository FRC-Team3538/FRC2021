#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Solenoid.h>
#include <ctre/Phoenix.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/PWMTalonSRX.h>
#include <AHRS.h>
#include <iostream>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
// #include "rev/CANSparkMax.h"

#include "ExternalDeviceProvider.hpp"

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
  WPI_TalonFX &motorLeft1;
  WPI_TalonFX &motorLeft2;

  WPI_TalonFX &motorRight1;
  WPI_TalonFX &motorRight2;

  Solenoid solenoidShifter{8};

  // Encoder Scale Factor (Inches)/(Pulse)
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
  double prevError_rel = 0.0;
  double iAcc = 0.0;

  bool oneShotAngle = false;

#define KP_ROTATIONV (SmartDashboard::GetNumber("KP_ROTATIONV", 0.018))
#define KI_ROTATIONV (SmartDashboard::GetNumber("KI_ROTATIONV", 0.000045)) //275
#define KD_ROTATIONV (SmartDashboard::GetNumber("KD_ROTATIONV", 0.001))    //15

#define KP_ROTATION (SmartDashboard::GetNumber("KP_ROTATION", 0.022))
#define KI_ROTATION (SmartDashboard::GetNumber("KI_ROTATION", 0.0002)) //275
#define KD_ROTATION (SmartDashboard::GetNumber("KD_ROTATION", 0.0015))    //15

#define KP_FORWARD (SmartDashboard::GetNumber("KP_FORWARD", 0.021))
#define KI_FORWARD (SmartDashboard::GetNumber("KI_FORWARD", 0.00050))
#define KD_FORWARD (SmartDashboard::GetNumber("KD_FORWARD", 0.0))

#define KP_FORWARDGYRO (SmartDashboard::GetNumber("KP_FORWARDGYRO", 0.008)) //75
#define KI_FORWARDGYRO (SmartDashboard::GetNumber("KI_FORWARDGYRO", 0.0))
#define KD_FORWARDGYRO (SmartDashboard::GetNumber("KD_FORWARDGYRO", 0.0))

  SendableChooser<std::string> chooseDriveLimit;
  const std::string sLimited = "Normal";
  const std::string sUnlimited = "Override";

public:
  // Default Constructor
  Drivebase(ExternalDeviceProvider &xdp): 
  motorLeft1(xdp.driveLeft1),
  motorLeft2(xdp.driveLeft2),
  motorRight1(xdp.driveRight1),
  motorRight2(xdp.driveRight2),
  navx(xdp.navx)
  {
    Configure();
  }

  void Configure();

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

  bool VisionAim(double forward, double degrees, double tolerance);
  void DriveForward(double distance, double maxoutput = 1.0);
  void TurnAbs(double degrees, double maxoutput = 1.0);
  bool TurnRel(double degrees, double tolerance);
  void SetMaxSpeed();

  AHRS &navx;

  //MotionMagisk * magiskR1 = new MotionMagisk( motorRight1, MotionMagisk::WaypointFile::backRockR );
};