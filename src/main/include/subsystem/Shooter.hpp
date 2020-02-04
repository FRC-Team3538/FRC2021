#pragma once

#include <ctre/Phoenix.h>
#include <frc/PWMTalonSRX.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Solenoid.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SendableChooser.h>

using namespace frc;
using namespace ctre::phoenix::motorcontrol::can;

class Shooter
{
private:
  //CTRE CAN
  WPI_TalonFX flywheel{6};
  WPI_TalonFX flywheelB{7};
  WPI_VictorSPX motorIntake{8};
  WPI_VictorSPX motorIndexer{9};
  WPI_TalonSRX motorFeeder{10}; //Actually a victor
  WPI_TalonSRX motorHood{11};

  Solenoid solenoidIntake{1};
  Solenoid solenoidHood{3};

  Timer shootDelay;
  bool shootOS = false;
  double dist = 0.0;
  bool distOS = false;

  SendableChooser<std::string> chooseShooterMode;
  const std::string sManualMode = "Manual";
  const std::string sAutoMode = "Auto";

  bool manualMode = false;
  double shootSpeed = 0.0;
  int shootCounter = 0;

  // Degrees / Pulses
  const double kScaleFactor = 360.0 / 8056.0;
  const double kScaleFactorFly = (1.0 / 2048);

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
  void SetIntake(double speed);
  void IntakeDeploy(); // TODO
  void IntakeRetract();
  void SetIndexer(double speed);

  void SetHood(double input);
  bool GetModeChooser();
  void ManualShoot(double inputFly, double inputKick);

  void StopShooter();


  void UpdateSmartdash();
};