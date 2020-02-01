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

   chooseShooterMode.SetDefaultOption(sAutoMode, sAutoMode);
   chooseShooterMode.AddOption(sManualMode, sManualMode);

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
   shootSpeed = 0.0;
}

void Shooter::SetShooterDistance(double distance)
{
   if (distance < 0.0)
   {
      motorFeeder.Set(0.0);
      flywheel.Set(0.0);
      shootSpeed = 0.0;
      return;
   }

   if(distance < 188)
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
}

void Shooter::IntakeDeploy()
{
   solenoidIntake.Set(true);
}

void Shooter::IntakeRetract()
{
   solenoidIntake.Set(false);
}

void Shooter::SetIndexer(double speed)
{
   motorIndexer.Set(speed);
}

void Shooter::SetHood(double input)
{
      motorHood.Set(input);
}

bool Shooter::GetModeChooser()
{
   return manualMode;
}

void Shooter::ManualShoot(double inputFly, double inputKick)
{
   flywheel.Set(inputFly);
   motorFeeder.Set(inputKick);
}

void Shooter::UpdateSmartdash()
{
   SmartDashboard::PutData("_ShooterMode", &chooseShooterMode);
   manualMode = (chooseShooterMode.GetSelected() == sManualMode);

    SmartDashboard::PutNumber("Shoot RPM Cmd", shootSpeed);
    SmartDashboard::PutNumber("Shoot RPM Act", flywheel.GetSelectedSensorVelocity() * kScaleFactorFly * 600.0);
    SmartDashboard::PutNumber("Intake", motorIntake.Get());
    SmartDashboard::PutNumber("Indexer", motorIndexer.Get());
    SmartDashboard::PutNumber("Feeder", motorFeeder.Get());
    SmartDashboard::PutNumber("Hood", motorHood.Get());

    SmartDashboard::PutBoolean("Solenoid Intake", solenoidIntake.Get());
    SmartDashboard::PutBoolean("Solenoid Hood", solenoidHood.Get());
}
