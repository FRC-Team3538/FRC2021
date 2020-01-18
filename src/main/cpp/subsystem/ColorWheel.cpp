#include "subsystem/ColorWheel.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

// Configure Hardware Settings
ColorWheel::ColorWheel()
{
  
}

// Stop all motors
void ColorWheel::Stop()
{
    motorColorWheel.StopMotor();
}

void ColorWheel::SetColorWheel(double speed)
{
   if(solenoidColorWheel.Get())
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
}
