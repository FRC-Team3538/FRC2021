#pragma once

#include <ctre/Phoenix.h>
#include <frc/PWMTalonSRX.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Solenoid.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <frc/DigitalInput.h>
#include "rev/CANSparkMax.h"

using namespace frc;
using namespace ctre::phoenix::motorcontrol::can;

class Shooter
{
private:
  //CTRE CAN
  WPI_TalonFX flywheel{6};
  WPI_TalonFX flywheelB{7};
  WPI_TalonFX motorIntake{8};

  WPI_VictorSPX motorIndexer{9};
  WPI_TalonFX motorIndexerB{15};
  WPI_VictorSPX motorIndexerC{16};

  rev::CANSparkMax sparkIndexerB{15, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax sparkIndexerC{16, rev::CANSparkMax::MotorType::kBrushless};

  WPI_TalonSRX motorFeeder{10};
  WPI_VictorSPX motorHood{11};

  Solenoid solenoidIntake{0};
  Solenoid solenoidHood{1};

  Timer shootDelay;
  bool shootOS = false;
  double dist = 0.0;
  bool distOS = false;

  double shootSpeed = 0.0;
  double shootAngle = 0.0;
  int shootCounter = 0;

#define OMNISPEED (SmartDashboard::GetNumber("Omni Speed", 0.75))
#define FIRSTBRUSH (SmartDashboard::GetNumber("First Brush Speed", 1.0))
#define SECONDBRUSH (SmartDashboard::GetNumber("Second Brush Speed", 1.0))

  // Degrees / Pulses
  const double kScaleFactor = 360.0 / 8056.0;
  const double kScaleFactorFly = (1.0 / 2048);
  const double kScaleFactorHood = (2.0 / 5.0) * (360.0 / 8096.0);

  DutyCycleEncoder hoodEncAbs{0};

  double iAcc = 0;
  double prevError_rel = 0;
  const double kPHood = 0.0250;
  const double kIHood = 0.000016;

  DigitalInput hoodZeroSw{3};
  Encoder hoodEncQuad{1, 2};

public:
  // Default Constructor

  enum ShooterMode
  {
    Angle = 0,
    Percent
  };

  Shooter();

  void Stop();
  void SetShooterDistanceTwo(double distance);
  void SetShooterDistanceThree(double distance);
  void SetVelocity(double velocity);
  double GetVelocity();
  void SetIntake(double speed);
  void SetFeeder(double speed);
  void SetShooter(double speed);
  void IntakeDeploy(); // TODO
  void IntakeRetract();
  void SetIndexer(double speed);

  void SetHood(double input);
  void SetHoodAngle(double angle);
  void SetHoodLock(bool lock);

  double GetHoodAngle();
  bool GetModeChooser();
  void ManualShoot(double inputFly, double inputKick);

  void StopShooter();
  void ResetHood();

  void UpdateSmartdash();

  void Init();
};