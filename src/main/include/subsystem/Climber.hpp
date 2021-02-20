#pragma once

#include <ctre/Phoenix.h>
#include <frc/PWMTalonSRX.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Solenoid.h>

#include "lib/Loggable.hpp"
#include <UDPLogger.hpp>

using namespace frc;
using namespace ctre::phoenix::motorcontrol::can;

class Climber
{
  private:

    // CTRE CAN
    WPI_TalonFX motorClimber0{ 12 };
    WPI_VictorSPX motorClimber1{ 13 };

    // Solenoids
    Solenoid solenoidClimber{ 2 };
    Solenoid climbBrake{ 3 };

  public:
    // Default Constructor
    Climber()
    {
      Configure();
    };

    void Configure();
    
    void Log(UDPLogger &logger)
  {
    logger.LogExternalDevice(motorClimber0);
    logger.LogExternalDevice(motorClimber1);
  }


    void Stop();
    void SetClimber(double speed);
    void ClimberDeploy();
    void ClimberRetract();

    void UpdateSmartdash();
};
