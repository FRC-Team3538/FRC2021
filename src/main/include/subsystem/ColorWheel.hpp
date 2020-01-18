#pragma once

#include <ctre/Phoenix.h>
#include <frc/PWMTalonSRX.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Solenoid.h>

using namespace frc;
using namespace ctre::phoenix::motorcontrol::can;

class ColorWheel
{
  private:

    WPI_TalonSRX motorColorWheel {14};

    // Solenoids
    Solenoid solenoidColorWheel{4};

  public:
    // Default Constructor
    ColorWheel();

    void Stop();
    void SetColorWheel(double speed);

    void ColorWheelDeploy();
    void ColorWheelRetract();

    void UpdateSmartdash();
};