#pragma once

#include <ctre/Phoenix.h>
#include <frc/PWMVictorSPX.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Solenoid.h>
#include <frc/DriverStation.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include "frc/AddressableLED.h"

using namespace frc;
using namespace ctre::phoenix::motorcontrol::can;

class ColorWheel
{
private:
  WPI_VictorSPX motorColorWheel{14};

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

  enum colors
  {
    blue,
    green,
    red,
    yellow,
    unknown,
    nothing,
  };

  colors targetColor;
  colors detectedColor;
  double confidence = 0.0;
  bool blueDetected = false;
  int spinCounter = 0;

  //LED Strip
  static constexpr int kLength = 45;
  static constexpr int kMiddleEnd = 31; //16-31 = middle section
  static constexpr int kMiddleStart = 16;

  frc::AddressableLED m_led{9};
  std::array<frc::AddressableLED::LEDData, kLength>
      m_ledBuffer; // Reuse the buffer
  // Store what the last hue of the first pixel is
  int firstPixelHue = 0;

public:
  // Default Constructor
  ColorWheel();

  void Stop();
  void SetColorWheel(double speed);
  void AutoColorWheel();

  void ColorWheelDeploy();
  void ColorWheelRetract();

  void ReadColorWheel();
  void UpdateSmartdash();
};