#include "subsystem/Climber.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

// Configure Hardware Settings
Climber::Climber()
{
   motorClimber0.ConfigFactoryDefault();
   motorClimber1.ConfigFactoryDefault();
   motorClimber0.SetNeutralMode(NeutralMode::Brake);
}

// Stop all motors
void Climber::Stop()
{
   motorClimber0.StopMotor();
   motorClimber1.StopMotor();
}

void Climber::SetClimber(double speed)
{
   if(speed == 0.0)
   {
      climbBrake.Set(true);
   }
   else
   {
      climbBrake.Set(false);
   }
   
   motorClimber0.Set(speed);
   motorClimber1.Set(speed);
}

void Climber::ClimberDeploy()
{
   solenoidClimber.Set(true);
}

void Climber::ClimberRetract()
{
   solenoidClimber.Set(false);
}

void Climber::UpdateSmartdash()
{
   SmartDashboard::PutNumber("Climber", motorClimber0.Get());
   SmartDashboard::PutBoolean("Solenoid Climber", solenoidClimber.Get());
}
