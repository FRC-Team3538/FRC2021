#pragma once

#include <ctre/Phoenix.h>
#include <frc/PWMTalonSRX.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Solenoid.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"

using namespace frc;
using namespace ctre::phoenix::motorcontrol::can;

class ColorWheel
{
  private:

    WPI_TalonSRX motorColorWheel {14};

    // Solenoids
    Solenoid solenoidColorWheel{4};

    //Color Sensor
    static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
    rev::ColorSensorV3 m_colorSensor{i2cPort};
    rev::ColorMatch m_colorMatcher;

    //Color Presets
    static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
    static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
    static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
    static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);

  public:
    // Default Constructor
    ColorWheel();

    void Stop();
    void SetColorWheel(double speed);

    void ColorWheelDeploy();
    void ColorWheelRetract();

    void UpdateSmartdash();
};