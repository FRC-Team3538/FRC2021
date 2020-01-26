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

void ColorWheel::AutoColorWheel()
{
   double wheelSpeed = 0.5;
   ReadColorWheel();

   switch (targetColor)
   {
   case blue:
      if (detectedColor == blue)
         wheelSpeed = 0.0;
      break;
   case green:
      if (detectedColor == green)
         wheelSpeed = 0.0;
      break;
   case red:
      if (detectedColor == red)
         wheelSpeed = 0.0;
      break;
   case yellow:
      if (detectedColor == yellow)
         wheelSpeed = 0.0;
      break;
   case unknown:
      if (detectedColor == unknown)
         wheelSpeed = 0.0;
      break;
   case nothing:
      if (detectedColor == blue)
      {
         blueDetected = true;
      }
      else if (detectedColor == green && blueDetected)
      {
         spinCounter++;
         blueDetected = false;
      }
      if (spinCounter >= 7)
      {
         wheelSpeed = 0.0;
      }
      break;
   }

   SetColorWheel(wheelSpeed);
}

void ColorWheel::ColorWheelDeploy()
{
   solenoidColorWheel.Set(true);
}

void ColorWheel::ColorWheelRetract()
{
   solenoidColorWheel.Set(false);
}

void ColorWheel::ReadColorWheel()
{
   Color Col = m_colorSensor.GetColor();
   confidence = 0.0;
   Color matchedColor = m_colorMatcher.MatchClosestColor(Col, confidence);

   if (matchedColor == kBlueTarget)
   {
      detectedColor = blue;
   }
   else if (matchedColor == kRedTarget)
   {
      detectedColor = red;
   }
   else if (matchedColor == kGreenTarget)
   {
      detectedColor = green;
   }
   else if (matchedColor == kYellowTarget)
   {
      detectedColor = yellow;
   }
   else
   {
      detectedColor = unknown;
   }

   std::string gameData;
   gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
   if (gameData.length() > 0)
   {
      switch (gameData[0])
      {
      case 'B':
         //Blue case code
         targetColor = blue;
         break;
      case 'G':
         //Green case code
         targetColor = green;
         break;
      case 'R':
         //Red case code
         targetColor = red;
         break;
      case 'Y':
         //Yellow case code
         targetColor = yellow;
         break;
      default:
         //This is corrupt data
         targetColor = unknown;
         break;
      }
   }
   else
   {
      //Code for no data received yet
      targetColor = nothing;
   }
}

void ColorWheel::UpdateSmartdash()
{
   SmartDashboard::PutNumber("Motor ColorWheel", motorColorWheel.Get());
   SmartDashboard::PutBoolean("Solenoid ColorWheel", solenoidColorWheel.Get());

   ReadColorWheel();

   SmartDashboard::PutNumber("Confidence", confidence);

   switch (detectedColor)
   {
   case blue:
      SmartDashboard::PutString("Detected Color", "blue");
      break;
   case green:
      SmartDashboard::PutString("Detected Color", "green");
      break;
   case red:
      SmartDashboard::PutString("Detected Color", "red");
      break;
   case yellow:
      SmartDashboard::PutString("Detected Color", "yellow");
      break;
   case unknown:
      SmartDashboard::PutString("Detected Color", "unknown");
      break;
   case nothing:
      SmartDashboard::PutString("Detected Color", "nothing");
      break;
   }

   switch (targetColor)
   {
   case blue:
      SmartDashboard::PutString("Target Color", "blue");
      break;
   case green:
      SmartDashboard::PutString("Target Color", "green");
      break;
   case red:
      SmartDashboard::PutString("Target Color", "red");
      break;
   case yellow:
      SmartDashboard::PutString("Target Color", "yellow");
      break;
   case unknown:
      SmartDashboard::PutString("Target Color", "unknown");
      break;
   case nothing:
      SmartDashboard::PutString("Target Color", "nothing");
      break;
   }
   
   SmartDashboard::PutNumber("Spin Counter", spinCounter);
}
