#include "subsystem/ColorWheel.hpp"
#include <frc/util/Color.h>

#include <frc/smartdashboard/SmartDashboard.h>

// Configure Hardware Settings
void
ColorWheel::Configure()
{
   motorColorWheel.ConfigFactoryDefault();

   m_colorMatcher.AddColorMatch(kBlueTarget);
   m_colorMatcher.AddColorMatch(kGreenTarget);
   m_colorMatcher.AddColorMatch(kRedTarget);
   m_colorMatcher.AddColorMatch(kYellowTarget);

   m_led.SetLength(kLength);
   m_led.SetData(m_ledBuffer);
   m_led.Start();
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
      if (detectedColor == colors::blue)
         wheelSpeed = 0.0;
      break;
   case green:
      if (detectedColor == colors::green)
         wheelSpeed = 0.0;
      break;
   case red:
      if (detectedColor == colors::red)
         wheelSpeed = 0.0;
      break;
   case yellow:
      if (detectedColor == colors::yellow)
         wheelSpeed = 0.0;
      break;
   case unknown:
      if (detectedColor == colors::unknown)
         wheelSpeed = 0.0;
      break;
   case nothing:
      if (detectedColor == colors::blue)
      {
         blueDetected = true;
      }
      else if (detectedColor == colors::green && blueDetected)
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
      for (int i = kMiddleStart; i < kMiddleEnd; i++)
      {
         m_ledBuffer[i].SetRGB(0, 0, 255);
      }
      m_led.SetData(m_ledBuffer);
   }
   else if (matchedColor == kRedTarget)
   {
      detectedColor = red;
      for (int i = kMiddleStart; i < kMiddleEnd; i++)
      {
         m_ledBuffer[i].SetRGB(255, 0, 0);
      }
      m_led.SetData(m_ledBuffer);
   }
   else if (matchedColor == kGreenTarget)
   {
      detectedColor = green;
      for (int i = kMiddleStart; i < kMiddleEnd; i++)
      {
         m_ledBuffer[i].SetRGB(0, 255, 0);
      }
      m_led.SetData(m_ledBuffer);
   }
   else if (matchedColor == kYellowTarget)
   {
      detectedColor = yellow;
      for (int i = kMiddleStart; i < kMiddleEnd; i++)
      {
         m_ledBuffer[i].SetRGB(255, 255, 0);
      }
      m_led.SetData(m_ledBuffer);
   }
   else
   {
      detectedColor = unknown;
      for (int i = kMiddleStart; i < kMiddleEnd; i++)
      {
         m_ledBuffer[i].SetRGB(255, 255, 255);
      }
      m_led.SetData(m_ledBuffer);
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
           for (int i = 0; i < kMiddleStart; i++) 
           {
              m_ledBuffer[i].SetRGB(0, 0, 255);
           }
           for (int i = kMiddleEnd; i < kLength; i++)
           {
              m_ledBuffer[i].SetRGB(0, 0, 255);
           }
            m_led.SetData(m_ledBuffer);
         break;
      case 'G':
         //Green case code
         targetColor = green;
            for (int i = 0; i < kMiddleStart; i++) 
           {
              m_ledBuffer[i].SetRGB(0, 255, 0);
           }
           for (int i = kMiddleEnd; i < kLength; i++)
           {
              m_ledBuffer[i].SetRGB(0, 255, 0);
           }
            m_led.SetData(m_ledBuffer);
         break;
      case 'R':
         //Red case code
         targetColor = red;
            for (int i = 0; i < kMiddleStart; i++) 
           {
              m_ledBuffer[i].SetRGB(255, 0, 0);
           }
           for (int i = kMiddleEnd; i < kLength; i++)
           {
              m_ledBuffer[i].SetRGB(255, 0, 0);
           }
            m_led.SetData(m_ledBuffer);
         break;
      case 'Y':
         //Yellow case code
         targetColor = yellow;
            for (int i = 0; i < kMiddleStart; i++) 
           {
              m_ledBuffer[i].SetRGB(255, 255, 0);
           }
           for (int i = kMiddleEnd; i < kLength; i++)
           {
              m_ledBuffer[i].SetRGB(255, 255, 0);
           }
            m_led.SetData(m_ledBuffer);
         break;
      default:
         //This is corrupt data
         targetColor = unknown;
            for (int i = 0; i < kMiddleStart; i++) 
           {
              m_ledBuffer[i].SetRGB(255, 255, 255);
           }
           for (int i = kMiddleEnd; i < kLength; i++)
           {
              m_ledBuffer[i].SetRGB(255, 255, 255);
           }
            m_led.SetData(m_ledBuffer);
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
   SmartDashboard::PutNumber("ColorWheel", motorColorWheel.Get());
   SmartDashboard::PutBoolean("Solenoid ColWheel", solenoidColorWheel.Get());

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
