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

  chooseTestDevice.SetDefaultOption(sDrive, sDrive);
  chooseTestDevice.AddOption(sIntake, sIntake);
  chooseTestDevice.AddOption(sIndexer, sIndexer);
  chooseTestDevice.AddOption(sFeeder, sFeeder);
  chooseTestDevice.AddOption(sShooter, sShooter);
  chooseTestDevice.AddOption(sShooterVelocity, sShooterVelocity);
  chooseTestDevice.AddOption(sHood, sHood);
  chooseTestDevice.AddOption(sClimber, sClimber);
  chooseTestDevice.AddOption(sColorWheel, sColorWheel);
  chooseTestDevice.AddOption(sVision, sVision);
}

void Robot::RobotPeriodic()
{
  // Reset Encoders
  frc::SmartDashboard::PutNumber("Hood Angle", IO.shooter.GetHoodAngle());

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

void Robot::TestInit()
{
}

void Robot::TestPeriodic()
{

  // Input
  double forward = Deadband(IO.ds.Driver.GetY(GenericHID::kLeftHand) * -1.0, 0.05);
  double rotate = Deadband(IO.ds.Driver.GetX(GenericHID::kRightHand) * -1.0, 0.05);

  bool X = IO.ds.Driver.GetCrossButton();
  bool O = IO.ds.Driver.GetCircleButton();
  bool S = IO.ds.Driver.GetSquareButton();
  bool T = IO.ds.Driver.GetTriangleButton();

  //  Controller Type
  auto testdevice = chooseTestDevice.GetSelected();

  // Drive
  if (testdevice == sDrive)
  {
    if (IO.ds.Driver.GetUpButton())
    {
      //IO.drivebase.DriveForward(84.0, 0.20);
      IO.drivebase.Arcade(0.2, 0.0);
    }
    else if (IO.ds.Driver.GetDownButton())
    {
      //IO.drivebase.DriveForward(-84.0, 0.20);
      IO.drivebase.Arcade(-0.2, 0.0);
    }
    else if (IO.ds.Driver.GetLeftButton())
    {
      //IO.drivebase.TurnAbs(90.0, 0.20);
      IO.drivebase.Arcade(0.0, 0.2);
    }
    else if (IO.ds.Driver.GetRightButton())
    {
      //IO.drivebase.TurnAbs(-90.0, 0.20);
      IO.drivebase.Arcade(0.0, -0.2);
    }
    else
    {
      // Drive
      IO.drivebase.Arcade(forward, rotate);

      IO.drivebase.ResetEncoders();
      IO.drivebase.ResetGyro();
    }
  }

  // Intake
  if (testdevice == sIntake)
  {
    IO.shooter.SetIntake(forward);
    if (X)
    {
      IO.shooter.IntakeDeploy();
    }
    if (O)
    {
      IO.shooter.IntakeRetract();
    }
  }
  else
  {
    IO.shooter.SetIntake(0.0);
  }

  // Indexer
  if (testdevice == sIndexer)
  {
    IO.shooter.SetIndexer(forward);
  }
  else
  {
    IO.shooter.SetIndexer(0.0);
  }

  // Feeder
  if (testdevice == sFeeder)
  {
    IO.shooter.SetFeeder(forward);
  }
  else
  {
    IO.shooter.SetFeeder(0.0);
  }

  // Shooter
  if (testdevice == sShooter)
  {
    IO.shooter.SetShooter(forward);
  }
  else if (testdevice == sShooterVelocity)
  {
    if (X)
    {
      IO.shooter.SetVelocity(3500.0);
    }
    if (O)
    {
      IO.shooter.SetVelocity(2500.0);
    }
    if (S)
    {
      IO.shooter.SetVelocity(1500.0);
    }
    if (T)
    {
      IO.shooter.SetVelocity(0.0);
    }
  }
  else
  {
    IO.shooter.SetShooter(0.0);
  }

  // Hood
  if (testdevice == sHood)
  {
    IO.shooter.SetHood(forward);

    if (X)
    {
      IO.shooter.SetHoodAngle(0.0);
    }
    if (O)
    {
      IO.shooter.SetHoodAngle(90.0);
    }
    if (S)
    {
      IO.shooter.SetHoodAngle(30.0);
    }
    if (T)
    {
      IO.shooter.SetHoodAngle(60.0);
    }
  }
  else
  {
    IO.shooter.SetHood(0.0);
  }

  // Climber
  if (testdevice == sClimber)
  {
    IO.climber.SetClimber(forward);

    if (X)
    {
      IO.climber.ClimberDeploy();
    }
    if (O)
    {
      IO.climber.ClimberDeploy();
    }
  }
  else
  {
    IO.climber.SetClimber(0.0);
  }

  // Color Wheel
  if (testdevice == sColorWheel)
  {
    IO.colorWheel.SetColorWheel(forward);

    if (X)
    {
      IO.colorWheel.ColorWheelDeploy();
    }
    if (O)
    {
      IO.colorWheel.ColorWheelRetract();
    }
  }
  else
  {
    IO.colorWheel.SetColorWheel(0.0);
  }

  // Vision
  if (testdevice == sVision)
  {
    if (X)
    {
      IO.RJV.Run(IO.RJV.Pipe::TwoClose);
    }
    if (O)
    {
      IO.RJV.Run(IO.RJV.Pipe::ThreeClose);
    }
    if (S)
    {
      IO.RJV.Run(IO.RJV.Pipe::ThreeFar);
    }
    if (T)
    {
      IO.RJV.Run(IO.RJV.Pipe::LongShot);
    }
  }
  else
  {
    IO.RJV.Reset();
  }
}

void Robot::TeleopPeriodic()
{
  // Drive
  double forward = Deadband(IO.ds.Driver.GetY(GenericHID::kLeftHand) * -1.0, 0.05);
  double rotate = Deadband(IO.ds.Driver.GetX(GenericHID::kRightHand) * -1.0, 0.05);
  double indexer = 0.0; // This is also here now :)

  // Turn speed limiting
  if (!IO.ds.Driver.GetStickButton(GenericHID::kRightHand))
  {
    rotate *= kDriveTurnLimit;
  }

  // Shooter Manual Mode
  if (IO.shooter.GetModeChooser() == true)
  {
    IO.drivebase.Arcade(forward, rotate);

    // Hood Presets
    if (IO.ds.Operator.GetStickButton(frc::GenericHID::kRightHand))
    {
      if (IO.ds.Operator.GetDownButton())
        hoodAngle = 14.0;
      else if (IO.ds.Operator.GetUpButton())
        hoodAngle = 54.0;
      else if (IO.ds.Operator.GetLeftButton())
        hoodAngle = 52.0;
      else if (IO.ds.Operator.GetRightButton())
        hoodAngle = c;
    }

    // Manual Hood Control
    double hoodAnalog = Deadband(IO.ds.Operator.GetY(GenericHID::kRightHand) * -1.0, deadband);
    if ((hoodAnalog != 0.0 || hoodAngle < 0.0) && !IO.ds.Operator.GetStickButton(frc::GenericHID::kRightHand))
    {
      hoodAngle = -1.0;
      IO.shooter.SetHood(hoodAnalog);
      IO.shooter.ResetHood();
    }
    else if (hoodAngle >= 0.0)
    {
      IO.shooter.SetHoodAngle(hoodAngle);
    }
    else
    {
      IO.shooter.SetHoodAngle(IO.shooter.GetHoodAngle());
    }

    // Manual Shooting System
    bool atSpeed = (manualShootPercent > 1.0) && (abs(manualShootPercent - IO.shooter.GetVelocity()) < 150);

    bool atAngle = (abs(c) - IO.shooter.GetHoodAngle()) < 2.0;

    if ((IO.ds.Operator.GetTriangleButton() || IO.ds.Driver.GetTriangleButton()) && atSpeed && atAngle)
    {
      indexer = 0.8;
      IO.shooter.SetFeeder(1.0);
    }
    else
    {
      IO.shooter.SetFeeder(0.0);
    }

    if (IO.ds.Operator.GetCrossButton() || IO.ds.Driver.GetCrossButton())
    {
      if (std::abs(manualShootPercent) <= 1.0)
      {
        IO.shooter.SetShooter(manualShootPercent);
      }
      else
      {
        IO.shooter.SetVelocity(manualShootPercent);
      }
    }

    if (IO.ds.Operator.GetCircleButton() || IO.ds.Driver.GetCircleButton())
    {
      IO.shooter.SetShooter(0.0);
    }
    if (IO.ds.Driver.GetSquareButton())
    {
      data = IO.RJV.Run(IO.RJV.Pipe::TwoClose);
      if (data.filled)
      {
        IO.drivebase.TurnRel(data.angle, 0.5) ? tpCt++ : tpCt = 0;
      }
    }
    else
    {
      data.filled = false;
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
      if (IO.ds.Operator.GetCircleButton() || IO.ds.Driver.GetCircleButton())
      {
        IO.RJV.SetPipeline(IO.RJV.Pipe::ThreeClose);
      }
      else if (IO.ds.Operator.GetCrossButton() || IO.ds.Driver.GetCrossButton())
      {
        IO.RJV.SetPipeline(IO.RJV.Pipe::ThreeFar);
      }
      else if (IO.ds.Operator.GetTriangleButton() || IO.ds.Driver.GetTriangleButton())
      {
        IO.RJV.SetPipeline(IO.RJV.Pipe::LongShot);
      }
      else if (IO.ds.Operator.GetSquareButton() || IO.ds.Driver.GetSquareButton())
      {
        IO.RJV.SetPipeline(IO.RJV.Pipe::TwoClose);
      }
    }
    else if (IO.ds.Operator.GetCircleButton() || IO.ds.Driver.GetCircleButton())
    {
      indexer = indexerSpeed;
      data = IO.RJV.Run(IO.RJV.Pipe::ThreeClose);
      if (data.filled)
      {
        if (tpCt > 10)
        {
          IO.shooter.SetShooterDistanceThree(data.distance);
          IO.drivebase.Arcade(0.0, 0.0);
          if (picCt < 10)
          {
            IO.RJV.TakePicture(true);
            picCt++;
          }
          else
          {
            IO.RJV.TakePicture(false);
          }
        }
        else
        {
          IO.drivebase.TurnRel(data.angle, 0.5) ? tpCt++ : tpCt = 0;
          IO.shooter.SetVelocity(1000.0);
          picCt = 0;
        }
      }
      else
      {
        IO.drivebase.Arcade(0.0, 0.0);
      }
      
    }
    else if (IO.ds.Operator.GetCrossButton() || IO.ds.Driver.GetCrossButton())
    {
      indexer = indexerSpeed;
      data = IO.RJV.Run(IO.RJV.Pipe::ThreeFar);
      if (data.filled)
      {
        if (tpCt > 10)
        {
          IO.shooter.SetShooterDistanceThree(data.distance);
          IO.drivebase.Arcade(0.0, 0.0);
          if (picCt < 10)
          {
            IO.RJV.TakePicture(true);
            picCt++;
          }
          else
          {
            IO.RJV.TakePicture(false);
          }
        }
        else
        {
          IO.drivebase.TurnRel(data.angle, 0.5) ? tpCt++ : tpCt = 0;
          IO.shooter.SetVelocity(1000.0);
          picCt = 0;
        }
      }
    }
    else if (IO.ds.Operator.GetSquareButton() || IO.ds.Driver.GetSquareButton()) //Two Pointer
    {
      data = IO.RJV.Run(IO.RJV.Pipe::TwoClose);
      IO.shooter.SetVelocity(3000.0);
      if (data.filled)
      {
        if (tpCt > 5)
        {
          IO.shooter.SetShooterDistanceTwo(data.distance);
          indexer = indexerSpeed;
          IO.drivebase.Arcade(0.0, 0.0);
          if (picCt < 10)
          {
            IO.RJV.TakePicture(true);
            picCt++;
          }
          else
          {
            IO.RJV.TakePicture(false);
          }
        }
        else
        {
          IO.drivebase.TurnRel(data.angle, 0.5) ? tpCt++ : tpCt = 0;
          picCt = 0;
        }
      }
      else
      {
        IO.drivebase.Arcade(0.0, 0.0);
      }
      
    }
    else if (IO.ds.Operator.GetTriangleButton() || IO.ds.Driver.GetTriangleButton()) //Two Pointer
    {
      indexer = indexerSpeed;
      data = IO.RJV.Run(IO.RJV.Pipe::LongShot);
      if (data.filled)
      {
        if (tpCt > 5)
        {
          IO.shooter.SetShooterDistanceTwo(data.distance);
          IO.drivebase.Arcade(0.0, 0.0);
          if (picCt < 10)
          {
            IO.RJV.TakePicture(true);
            picCt++;
          }
          else
          {
            IO.RJV.TakePicture(false);
          }
        }
        else
        {
          IO.drivebase.TurnRel(data.angle, 0.4) ? tpCt++ : tpCt = 0;
          IO.shooter.SetVelocity(1000.0);
          picCt = 0;
        }
      }
    }
    else
    {
      data.filled = false;
      tpCt = 0;
      IO.drivebase.Arcade(0, 0);
      IO.shooter.StopShooter();
      IO.RJV.Reset();
      picCt = 0;
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
  intakeSpeed = Deadband(-intakeSpeed, deadband);
  IO.shooter.SetIntake(intakeSpeed * 0.8);

  if (intakeSpeed > 0.05)
  {
    indexer = indexerSpeed;
  }
  else if (intakeSpeed < -0.05)
  {
    indexer = -indexerSpeed;
  }

  IO.shooter.SetIndexer(indexer);

  // Climber
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

  // ColorWheel
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
  IO.shooter.UpdateSmartdash();
  switch (smartDashSkip)
  {
  case 0:
  {
    IO.drivebase.UpdateSmartdash();
    break;
  }

  case 5:
  {

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
  case 25:
  {
    SmartDashboard::PutData("_TestDevice", &chooseTestDevice);

    std::string sDF = "ShootManual";
    manualShootPercent = frc::SmartDashboard::GetNumber(sDF, manualShootPercent);
    frc::SmartDashboard::PutNumber(sDF, manualShootPercent);
    frc::SmartDashboard::SetPersistent(sDF);

    std::string sDFe = "Hood Target";
    c = frc::SmartDashboard::GetNumber(sDFe, 45.0);
    frc::SmartDashboard::PutNumber(sDFe, c);
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
