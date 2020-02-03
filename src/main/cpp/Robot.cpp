/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.hpp"
#include <iostream>
#include <frc/Threads.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
  IO.drivebase.ResetEncoders();
  IO.drivebase.ResetGyro();
  IO.RJV.Init();
}

void Robot::RobotPeriodic()
{
  // Reset Encoders
  bool btnPSDr = IO.ds.Driver.GetPSButton();
  if (btnPSDr)
  {
    IO.drivebase.ResetEncoders();
    IO.drivebase.ResetGyro();
    autoPrograms.Init();
  }

  // Vision
  IO.RJV.Periodic();

  // Update Smart Dash
  UpdateSD();

  std::string sDF = "Fly Output (Percent)";
  df = frc::SmartDashboard::GetNumber(sDF, 100.0);
  frc::SmartDashboard::PutNumber(sDF, df);
  frc::SmartDashboard::SetPersistent(sDF);
}

void Robot::AutonomousInit()
{
  IO.drivebase.ResetEncoders();
  IO.drivebase.ResetGyro();
  autoPrograms.Init();
}

void Robot::AutonomousPeriodic()
{
  autoPrograms.Run();
}

void Robot::TeleopInit()
{
  IO.drivebase.SetBrake();
}

void Robot::DisabledInit()
{
  IO.drivebase.SetCoast();
}

void Robot::TeleopPeriodic()
{

  // Drive
  double forward = Deadband(IO.ds.Driver.GetY(GenericHID::kLeftHand) * 1, 0.05);
  double rotate = Deadband(IO.ds.Driver.GetX(GenericHID::kRightHand) * 1.0, 0.05);
  double indexer = 0.0; // This is also here now :)

  // Turn speed limiting
  if (!IO.ds.Driver.GetStickButton(GenericHID::kRightHand))
  {
    rotate *= kDriveTurnLimit;
  }

  // Shooter Manual Mode
  if (IO.shooter.GetModeChooser() == true)
  {
    //Hood
    double hoodAnalog = Deadband(IO.ds.Operator.GetY(GenericHID::kRightHand) * -1, deadband);
    IO.shooter.SetHood(hoodAnalog);

    if (IO.ds.Operator.GetCircleButton() || IO.ds.Driver.GetCircleButton())
    {
      IO.shooter.ManualShoot(0.0, 0.0);
    }
    if (IO.ds.Operator.GetCrossButton() || IO.ds.Driver.GetCrossButton())
    {
      manualShootTimer.Start();
      if (manualShootTimer.Get() < 1.0)
      {
        IO.shooter.ManualShoot(1.0, 0.0);
      }
      else
      {
        indexer = 1.0;
        IO.shooter.ManualShoot(1.0, 1.0);
      }
    }
    else
    {
      manualShootTimer.Stop();
      manualShootTimer.Reset();
    }
  }
  else
  {
    if ((abs(forward) > 0.0) || (abs(rotate) > 0.0))
    {
      IO.drivebase.Arcade(forward, rotate);
      data.filled = false;
      tpCt = 0;
      IO.shooter.StopShooter();
      IO.RJV.Reset();
    }
    else if (IO.ds.Operator.GetCircleButton() || IO.ds.Driver.GetCircleButton())
    {
      //std::cout << IO.drivebase.TurnRel(data.angle) << std::endl;
      indexer = indexerSpeed;
      data = IO.RJV.Run(IO.RJV.ShotType::Three);
      if (data.filled)
      {
        if (tpCt > 10)
        {
          IO.shooter.SetShooterDistanceThree(data.distance);
          IO.drivebase.Arcade(0.0, 0.0);
        }
        else
        {
          IO.drivebase.TurnRel(data.angle, 0.5) ? tpCt++ : tpCt = 0;
          IO.shooter.SetVelocity(1500.0);
        }
      }
    }
    else if (IO.ds.Operator.GetCrossButton() || IO.ds.Driver.GetCrossButton()) //Two Pointer
    {
      indexer = indexerSpeed;
      data = IO.RJV.Run(IO.RJV.ShotType::Two);
      if (data.filled)
      {
        if (tpCt > 5)
        {
          IO.shooter.SetShooterDistanceTwo(data.distance);
          IO.drivebase.Arcade(0.0, 0.0);
        }
        else
        {
          IO.drivebase.TurnRel(data.angle, 0.4) ? tpCt++ : tpCt = 0;
          IO.shooter.SetVelocity(1000.0);
        }
      }
    }
    else if (IO.ds.Operator.GetSquareButton() || IO.ds.Driver.GetSquareButton())
    {
      IO.shooter.SetVelocity(df);
    }
    else
    {
      data.filled = false;
      tpCt = 0;
      IO.drivebase.Arcade(0, 0);
      IO.shooter.StopShooter();
      IO.RJV.Reset();
    }
  }
  // Shifting
  if (IO.ds.Driver.GetBumper(GenericHID::kLeftHand))
  {
    IO.drivebase.SetLowGear();
  }

  if (IO.ds.Driver.GetBumper(GenericHID::kRightHand))
  {
    IO.drivebase.SetHighGear();
  }

  // Shooter
  double leftTrigOp = IO.ds.Operator.GetTriggerAxis(GenericHID::kLeftHand);
  double rightTrigOp = IO.ds.Operator.GetTriggerAxis(GenericHID::kRightHand);
  double leftTrigDr = IO.ds.Driver.GetTriggerAxis(GenericHID::kLeftHand);
  double rightTrigDr = IO.ds.Driver.GetTriggerAxis(GenericHID::kRightHand);
  double intakeSpeed = leftTrigOp - rightTrigOp + leftTrigDr - rightTrigDr;
  intakeSpeed = Deadband(intakeSpeed, deadband);
  IO.shooter.SetIntake(intakeSpeed);

  if (abs(intakeSpeed) > 0.05)
  {
    indexer = indexerSpeed;
  }

  IO.shooter.SetIndexer(indexer);

  //Intake
  if (IO.ds.Operator.GetBumperPressed(GenericHID::kRightHand))
  {
  }

  //Climber
  if (IO.ds.Operator.GetRightButton())
  {
    IO.climber.ClimberDeploy();
  }
  if (IO.ds.Operator.GetLeftButton())
  {
    IO.climber.ClimberRetract();
  }
  double climbAnalog = Deadband(IO.ds.Operator.GetY(GenericHID::kLeftHand) * -1, deadband);
  IO.climber.SetClimber(climbAnalog);

  //ColorWheel
  if (IO.ds.Operator.GetUpButton())
  {
    IO.colorWheel.ColorWheelDeploy();
  }
  if (IO.ds.Operator.GetDownButton())
  {
    IO.colorWheel.ColorWheelRetract();
  }
  double colorAnalog = Deadband(IO.ds.Operator.GetX(GenericHID::kRightHand) * -1, deadband);
  IO.colorWheel.SetColorWheel(colorAnalog);

  if (IO.ds.Operator.GetStickButton(GenericHID::kRightHand))
  {
    IO.colorWheel.AutoColorWheel();
  }
}

double Robot::Deadband(double input, double deadband)
{
  if ((std::abs(input)) < deadband)
  {
    return 0.0;
  }
  else if (input > 0.95)
  {
    return 1.0;
  }
  else if (input < -0.95)
  {
    return -1.0;
  }
  else
  {
    return input;
  }
}

void Robot::UpdateSD()
{
  // Don't update smart dash every loop,
  // it causes watchdog warnings
  smartDashSkip %= 30;
  switch (smartDashSkip)
  {
    // case 0:
    // {
    //   IO.drivebase.UpdateSmartdash();
    //   break;
    // }

  case 5:
  {
    IO.shooter.UpdateSmartdash();
    break;
  }

  case 10:
  {
    IO.climber.UpdateSmartdash();
    break;
  }

  case 15:
  {
    IO.colorWheel.UpdateSmartdash();
    break;
  }
  case 20:
  {
    IO.RJV.UpdateSmartDash();
    break;
  }
  default:
  {
  }
  }

  // Critical
  autoPrograms.SmartDash();
  IO.ds.SmartDash();
  smartDashSkip++;
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
