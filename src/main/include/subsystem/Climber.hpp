#pragma once

#include <ctre/Phoenix.h>
#include <frc/PWMTalonSRX.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Solenoid.h>

using namespace frc;
using namespace ctre::phoenix::motorcontrol::can;

class Climber
{
  private:

    // CTRE CAN
    WPI_TalonFX motorClimber0 {12};
    WPI_VictorSPX motorClimber1 {13};

    // Solenoids
    Solenoid solenoidClimber{2};
    Solenoid climbBrake{3};

  public:
    // Default Constructor
    Climber();

    void Stop();
    void SetClimber(double speed);
    void ClimberDeploy();
    void ClimberRetract();

    void UpdateSmartdash();
};
