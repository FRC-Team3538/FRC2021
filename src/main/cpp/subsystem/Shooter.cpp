#include "subsystem/Shooter.hpp"

#include <frc/smartdashboard/SmartDashboard.h>
#include "frc/Preferences.h"

// Configure Hardware Settings
void
Shooter::Configure()
{
   flywheel.ConfigFactoryDefault();
   flywheelB.ConfigFactoryDefault();
   motorIntake.ConfigFactoryDefault();
   motorIndexer.ConfigFactoryDefault();
   motorIndexerB.ConfigFactoryDefault();
   motorIndexerC.ConfigFactoryDefault();
   motorFeeder.ConfigFactoryDefault();
   motorHood.ConfigFactoryDefault();

   sparkIndexerB.RestoreFactoryDefaults();
   sparkIndexerC.RestoreFactoryDefaults();

   sparkIndexerB.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
   sparkIndexerC.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

   flywheelB.Follow(flywheel);

   flywheel.ConfigVoltageCompSaturation(10.0);
   flywheelB.ConfigVoltageCompSaturation(10.0);

   flywheel.ConfigClosedloopRamp(0.4);
   flywheelB.ConfigClosedloopRamp(0.4);

   flywheel.SetInverted(true);
   flywheelB.SetInverted(false);

   flywheel.Config_kF(0, 0.056494409);
   flywheel.Config_kP(0, 0.25);
   flywheel.Config_kI(0, 0.0001);
   flywheel.Config_kD(0, 7.000);

   flywheel.Config_IntegralZone(0, 200.0);

   motorIntake.SetInverted(false);

   motorIndexer.SetInverted(false);  // Omni Rollers
   motorIndexerB.SetInverted(false); // First set of brushes (Rear)
   motorIndexerC.SetInverted(true);  // Second set of brushes (Front)

   motorFeeder.SetInverted(true);
   motorHood.SetInverted(true);

   flywheel.SetNeutralMode(NeutralMode::Coast);
   flywheelB.SetNeutralMode(NeutralMode::Coast);
   motorIntake.SetNeutralMode(NeutralMode::Brake);
   motorIndexer.SetNeutralMode(NeutralMode::Brake);
   motorIndexerB.SetNeutralMode(NeutralMode::Brake);
   motorIndexerC.SetNeutralMode(NeutralMode::Brake);
   motorFeeder.SetNeutralMode(NeutralMode::Brake);
   motorHood.SetNeutralMode(NeutralMode::Brake);

   hoodEncAbs.SetDistancePerRotation(360.0 / 2.5);
   hoodEncQuad.SetDistancePerPulse(45.0 / 630.0);
}

void Shooter::Init()
{
   flywheelB.Follow(flywheel);

   flywheel.ConfigVoltageCompSaturation(10.0);
   flywheelB.ConfigVoltageCompSaturation(10.0);

   flywheel.ConfigClosedloopRamp(0.4);
   flywheelB.ConfigClosedloopRamp(0.4);

   flywheel.SetInverted(true);
   flywheelB.SetInverted(false);
}

// Stop all motors
void Shooter::Stop()
{
   flywheel.StopMotor();
   flywheelB.StopMotor();
   motorIntake.StopMotor();
   motorIndexer.StopMotor();
   motorIndexerB.StopMotor();
   motorIndexerC.StopMotor();
   motorFeeder.StopMotor();
   sparkIndexerB.Set(0.0);
   sparkIndexerC.Set(0.0);
   shootSpeed = 0.0;
}

void Shooter::StopShooter()
{
   flywheel.StopMotor();
   flywheelB.StopMotor();
   motorFeeder.StopMotor();
   shootOS = false;
   distOS = false;
}

void Shooter::ResetHood()
{
   iAcc = 0.0;
}

double Shooter::GetVelocity()
{
   return flywheel.GetSelectedSensorVelocity() * kScaleFactorFly * 600.0;
}

void Shooter::SetShooterDistanceThree(double distance)
{
   if (!distOS || distance < 0.0)
   {
      dist = distance;
      distOS = true;
   }
   std::cout << dist << std::endl;

   if (dist < 0.0)
   {
      shootSpeed = 0.0;
      distOS = false;
   }
   else if (dist < 120)
   {
      shootSpeed = -2450;
      shootAngle = (0.0577 * dist) + 36.231;
   }
   else
   {
      shootSpeed = -3000;
      shootAngle = (-0.0452 * dist) + 57.451;
      //shootSpeed = (3.7015 * distance) + 2560.3; //2480.3 3.6415
   }
   flywheel.Set(ControlMode::Velocity, -((shootSpeed / kScaleFactorFly) / 600.0));
   SetHoodAngle(shootAngle);

   if (std::abs(std::abs(flywheel.GetSensorCollection().GetIntegratedSensorVelocity()) - std::abs(((shootSpeed / kScaleFactorFly) / 600.0))) < 800.0) // && (shootSpeed > 100.0))
   {
      shootCounter++;
   }
   else if (std::abs(std::abs(flywheel.GetSensorCollection().GetIntegratedSensorVelocity()) - std::abs(((shootSpeed / kScaleFactorFly) / 600.0))) > 2500.0) // || (shootSpeed < 100.0))
   {
      shootCounter = 0;
   }

   if (shootCounter > 3)
   {
      SetFeeder(1.0);
   }
   else
   {
      SetFeeder(0.0);
   }
}

void Shooter::SetShooterDistanceTwo(double distance)
{
   if (!distOS || distance < 0.0)
   {
      dist = distance;
      distOS = true;
   }
   std::cout << dist << std::endl;

   if (dist < 0.0)
   {
      shootSpeed = 0.0;
      distOS = false;
   }
   else if (dist < 240.0)
   {
      shootSpeed = -3000;
      shootAngle = -(0.0017 * pow(dist, 2)) + (0.6213 * dist) + 4.4679;
      //shootAngle = -(0.0007 * pow(dist, 2)) + (0.3637 * dist) + 20.901; //0.0011
   }
   else
   {
      shootSpeed = (0.0021 * pow(dist, 2)) + (2.3603 * dist) + 2600.8;
      //shootSpeed = (3.7015 * distance) + 2560.3; //2480.3 3.6415
   }
   flywheel.Set(ControlMode::Velocity, -((shootSpeed / kScaleFactorFly) / 600.0));
   SetHoodAngle(shootAngle);

   if (std::abs(std::abs(flywheel.GetSensorCollection().GetIntegratedSensorVelocity()) - std::abs(((shootSpeed / kScaleFactorFly) / 600.0))) < 1000.0) // && (shootSpeed > 100.0))
   {
      shootCounter++;
   }
   else if (std::abs(std::abs(flywheel.GetSensorCollection().GetIntegratedSensorVelocity()) - std::abs(((shootSpeed / kScaleFactorFly) / 600.0))) > 2500.0) // || (shootSpeed < 100.0))
   {
      shootCounter = 0;
   }

   if (shootCounter > 3 && (abs(GetHoodAngle() - shootAngle) < 3.0))
   {
      SetFeeder(1.0);
   }
   else
   {
      SetFeeder(0.0);
   }
}

void Shooter::SetVelocity(double velocity)
{
   shootSpeed = velocity;
   flywheel.Set(ControlMode::Velocity, ((velocity / kScaleFactorFly) / 600.0));
   flywheelB.Follow(flywheel);
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
   motorIndexerB.Set(ControlMode::PercentOutput, speed * FIRSTBRUSH);
   motorIndexer.Set(ControlMode::PercentOutput, speed * OMNISPEED);
   motorIndexerC.Set(ControlMode::PercentOutput, speed * SECONDBRUSH);
   sparkIndexerB.Set(speed * FIRSTBRUSH);
   sparkIndexerC.Set(speed * SECONDBRUSH);

}

void Shooter::SetFeeder(double speed)
{
   //bool lockHood = (fabs(speed) > 0.05);
   //solenoidHood.Set(lockHood);

   motorFeeder.Set(speed);
}

void Shooter::SetHoodLock(bool lock)
{
   //solenoidHood.Set(lock);
}

void Shooter::SetShooter(double speed)
{
   flywheel.Set(speed);
   flywheelB.Set(speed);
}

void Shooter::SetHood(double input)
{
   shootAngle = GetHoodAngle();
   // Hood Lock

   bool lockHood = (fabs(input) < 0.05);
   solenoidHood.Set(lockHood);
   if (lockHood)
   {
      motorHood.Set(0.0);
      return;
   }

   // Manual Mode

   // Soft Limits
   if (!hoodZeroSw.Get() && input < 0.0)
   {
      motorHood.Set(0.0);
   }
   else if ((GetHoodAngle() < 25.0) && (input < 0.0))
   {
      motorHood.Set(0.4 * input);
   }
   else if ((GetHoodAngle() > 90.0) && (input > 0.0))
   {
      motorHood.Set(0.0);
   }
   else if ((GetHoodAngle() > 81.0) && (input > 0.0))
   {
      motorHood.Set(0.4 * input);
   }
   else
   {
      motorHood.Set(input);
   }
}

void Shooter::SetHoodAngle(double angle)
{
   shootAngle = angle;

   // Soft Limits
   if (angle < 10.0)
   {
      angle = 10.0;
   }
   else if (angle > 90.0)
   {
      angle = 90.0;
   }

   double error = GetHoodAngle() - angle;

   // Accumulator
   if (std::fabs(error) < 5.0)
   {
      iAcc += error / 0.02;
   }
   else
   {
      iAcc = 0;
   }

   // PID Control
   if (std::fabs(error) > 0.3)
   {
      SetHood(-((error * kPHood) + (iAcc * kIHood)));
   }
   else
   {
      iAcc = 0;
      SetHood(0.0);
   }
}

double Shooter::GetHoodAngle()
{
   auto pref = frc::Preferences::GetInstance();
   auto offset = pref->GetDouble("HoodOffset", 0.0);
   pref->PutDouble("HoodOffset", offset);

   // Zero Switch
   if (!hoodZeroSw.Get())
   {
      hoodEncQuad.Reset();
   }

   return (hoodEncQuad.GetDistance() + offset);
   //return (-hoodEncAbs.GetDistance() + offset);
}

void Shooter::ManualShoot(double inputFly, double inputKick)
{
   flywheel.Set(inputFly);
   SetFeeder(inputKick);
}

void Shooter::UpdateSmartdash()
{

   SmartDashboard::PutNumber("Shoot RPM Cmd", shootSpeed);
   SmartDashboard::PutNumber("Shoot Angle Cmd", shootAngle);
   SmartDashboard::PutNumber("Shoot RPM Act", GetVelocity());

   SmartDashboard::PutNumber("Intake", motorIntake.Get());
   SmartDashboard::PutNumber("Indexer", motorIndexer.Get());
   SmartDashboard::PutNumber("Feeder", motorFeeder.Get());
   SmartDashboard::PutNumber("Hood", motorHood.Get());

   SmartDashboard::PutBoolean("Solenoid Intake", solenoidIntake.Get());
   SmartDashboard::PutBoolean("Solenoid Hood", solenoidHood.Get());

   SmartDashboard::PutBoolean("Hood Switch", !hoodZeroSw.Get());

   SmartDashboard::PutNumber("Hood Angle", GetHoodAngle());

   SmartDashboard::PutNumber("Omni Speed", OMNISPEED);
   SmartDashboard::PutNumber("First Brush Speed", FIRSTBRUSH);
   SmartDashboard::PutNumber("Second Brush Speed", SECONDBRUSH);
}
