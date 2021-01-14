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
    Solenoid solenoidClimber{2};
    Solenoid climbBrake{3};

  public:
    // Default Constructor
    Climber(ExternalDeviceProvider &xdp):
    motorClimber0(xdp.motorClimber0),
    motorClimber1(xdp.motorClimber1) 
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
