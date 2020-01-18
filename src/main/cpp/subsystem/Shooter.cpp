#include "subsystem/Shooter.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

// Configure Hardware Settings
Shooter::Shooter()
{
  motorShooter1.Follow(motorShooter0);
  motorShooter0.ConfigVoltageCompSaturation(10.0);
  motorShooter1.ConfigVoltageCompSaturation(10.0);
  shootDelay.Start();
}

// Stop all motors
void Shooter::Stop()
{
    motorShooter0.StopMotor();
    motorShooter1.StopMotor();
    motorIntake.StopMotor();
    motorIndexer.StopMotor();
    motorFeeder.StopMotor();
}

void Shooter::SetShooterDistance(double distance)
{
   if(distance == 0.0)
   {
      motorFeeder.Set(0.0);
      motorShooter0.Set(0.0);
      shootDelay.Reset();
   }
   else
   {
      double power = 0.03990359168 * (distance - 109.0) + 45.0;
      motorShooter0.Set(power);
   }
   
   if(shootDelay.Get() > 5.0)
   {
      motorFeeder.Set(0.5);
   }

   solenoidHood.Set(false);
}

void Shooter::SetIntake(double speed)
{
   motorIntake.Set(speed);
   
   bool deploy = abs(speed)>0.03;
   solenoidIntake.Set(deploy);
}

void Shooter::SetIndexer(double speed)
{
   motorIndexer.Set(speed);
}

void Shooter::UpdateSmartdash()
{
    SmartDashboard::PutNumber("Shooter", motorShooter0.Get());
    SmartDashboard::PutNumber("Intake", motorIntake.Get());
    SmartDashboard::PutNumber("Indexer", motorIndexer.Get());
    SmartDashboard::PutNumber("Feeder", motorFeeder.Get());
}
