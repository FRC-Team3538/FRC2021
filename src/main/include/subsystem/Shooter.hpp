#pragma once

#include <ctre/Phoenix.h>
#include <frc/PWMTalonSRX.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Solenoid.h>
#include <frc/Timer.h>

using namespace frc;
using namespace ctre::phoenix::motorcontrol::can;

class Shooter
{
  private:

    //CTRE CAN
    WPI_TalonFX motorShooter0 {6};
    WPI_TalonFX motorShooter1 {7};
    WPI_VictorSPX motorIntake {8};
    WPI_VictorSPX motorIndexer {9};
    WPI_VictorSPX motorFeeder {10};

    Solenoid solenoidIntake{1};
    Solenoid solenoidHood{3};

    Timer shootDelay;

  public:
    // Default Constructor
    Shooter();

    void Stop();
    void SetShooterDistance(double distance);
    void SetIntake(double speed);
    void SetIndexer(double speed);

    void UpdateSmartdash();
};