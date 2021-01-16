#pragma once

#include <ctre/Phoenix.h>
#include <frc/PWMTalonSRX.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Solenoid.h>

#include "ExternalDeviceProvider.hpp"

using namespace frc;
using namespace ctre::phoenix::motorcontrol::can;

class Climber
{
  private:

    // CTRE CAN
    WPI_TalonFX &motorClimber0;
    WPI_VictorSPX &motorClimber1;

    // Solenoids
    Solenoid &solenoidClimber;
    Solenoid &climbBrake;

  public:
    // Default Constructor
    Climber(ExternalDeviceProvider &xdp):
    motorClimber0(xdp.motorClimber0),
    motorClimber1(xdp.motorClimber1),
    solenoidClimber(xdp.solenoidClimber),
    climbBrake(xdp.climbBrake)
    {
      Configure();
    };

    void Configure();

    void Stop();
    void SetClimber(double speed);
    void ClimberDeploy();
    void ClimberRetract();

    void UpdateSmartdash();
};
