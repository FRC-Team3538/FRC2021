#include "subsystem/ColorWheel.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

// Configure Hardware Settings
ColorWheel::ColorWheel()
{
   m_colorMatcher.AddColorMatch(kBlueTarget);
   m_colorMatcher.AddColorMatch(kGreenTarget);
   m_colorMatcher.AddColorMatch(kRedTarget);
   m_colorMatcher.AddColorMatch(kYellowTarget);
}

// Stop all motors
void ColorWheel::Stop()
{
   motorColorWheel.StopMotor();
}

void ColorWheel::SetColorWheel(double speed)
{
   if (solenoidColorWheel.Get())
   {
      motorColorWheel.Set(speed);
   }
   else
   {
      motorColorWheel.Set(0.0);
   }
}

void ColorWheel::ColorWheelDeploy()
{
   solenoidColorWheel.Set(true);
}

void ColorWheel::ColorWheelRetract()
{
   solenoidColorWheel.Set(false);
}

void ColorWheel::UpdateSmartdash()
{
   SmartDashboard::PutNumber("Motor ColorWheel", motorColorWheel.Get());
   SmartDashboard::PutBoolean("Solenoid ColorWheel", solenoidColorWheel.Get());
   Color detectedColor = m_colorSensor.GetColor();

   std::string colorString;
    double confidence = 0.0;
    Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

    if (matchedColor == kBlueTarget) {
      colorString = "Blue";
    } else if (matchedColor == kRedTarget) {
      colorString = "Red";
    } else if (matchedColor == kGreenTarget) {
      colorString = "Green";
    } else if (matchedColor == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard::PutNumber("Red", detectedColor.red);
    SmartDashboard::PutNumber("Green", detectedColor.green);
    SmartDashboard::PutNumber("Blue", detectedColor.blue);
    SmartDashboard::PutNumber("Confidence", confidence);
    SmartDashboard::PutString("Detected Color", colorString);

}
