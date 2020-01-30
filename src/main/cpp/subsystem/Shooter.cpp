#include "subsystem/Shooter.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

// Configure Hardware Settings
Shooter::Shooter()
{
   flywheel.ConfigFactoryDefault();
   flywheelB.ConfigFactoryDefault();

   flywheelB.Follow(flywheel);

   flywheel.ConfigVoltageCompSaturation(10.0);
   flywheelB.ConfigVoltageCompSaturation(10.0);

   flywheel.Config_kF(0, 0.05643219); //0.05562068
   flywheel.Config_kP(0, 0.25);       //0.35
   flywheel.Config_kI(0, 0.0);        // 0.000
   flywheel.Config_kD(0, 7.000);      //6.0

   flywheel.ConfigClosedloopRamp(0.3);
   flywheelB.ConfigClosedloopRamp(0.3);

   flywheel.SetInverted(false);
   flywheelB.SetInverted(true);

   chooseShooterMode.SetDefaultOption(sManualMode, sManualMode);
   chooseShooterMode.AddOption(sAutoMode, sAutoMode);

}

// Stop all motors
void Shooter::Stop()
{
   flywheel.StopMotor();
   flywheelB.StopMotor();
   motorIntake.StopMotor();
   motorIndexer.StopMotor();
   motorFeeder.StopMotor();
   motorHood.StopMotor();
}

void Shooter::SetShooterDistance(double distance)
{
   if (distance < 0.0)
   {
      motorFeeder.Set(0.0);
      flywheel.Set(0.0);
   }
   else if(distance < 188)
   {
      shootSpeed = (4.2753 * (distance)) + 2041.9;
   }
   else
   {
      shootSpeed = (3.5415 * distance) + 2480.3;
   }
   flywheel.Set(ControlMode::Velocity, ((shootSpeed / kScaleFactorFly) / 600.0));   

   // TODO Set Flyweel hood postion

   // TODO Activate feeder and conveyor when setpoint is reached.
}

void Shooter::SetIntake(double speed)
{
   motorIntake.Set(speed);

   bool deploy = abs(speed) > 0.03;
   solenoidIntake.Set(deploy);
}

void Shooter::SetIndexer(double speed)
{
   motorIndexer.Set(speed);
}

void Shooter::SetHood(double input)
{

   if (manualMode)
   {
      motorHood.Set(input);
   }
   else
   {
      motorHood.Set(0.0);
   }

}

void Shooter::UpdateSmartdash()
{
   SmartDashboard::PutData("_ShooterMode", &chooseShooterMode);
   manualMode = (chooseShooterMode.GetSelected() == sManualMode);

    SmartDashboard::PutNumber("Shooter", flywheel.Get());
    SmartDashboard::PutNumber("Intake", motorIntake.Get());
    SmartDashboard::PutNumber("Indexer", motorIndexer.Get());
    SmartDashboard::PutNumber("Feeder", motorFeeder.Get());
    SmartDashboard::PutNumber("Hood", motorHood.Get());

    SmartDashboard::PutBoolean("Solenoid Intake", solenoidIntake.Get());
    SmartDashboard::PutBoolean("Solenoid Hood", solenoidHood.Get());
}
