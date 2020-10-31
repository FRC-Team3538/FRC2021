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
  IO.shooter.Init();
  autoPrograms.Init();
  IO.drivebase.SetBrake();
}

void Robot::AutonomousPeriodic()
{
  autoPrograms.Run();
}

void Robot::TeleopInit()
{
  IO.drivebase.SetBrake();
  IO.shooter.Init();
}

void Robot::DisabledInit()
{
  disableTimer.Reset();
  disableTimer.Start();
}

void Robot::DisabledPeriodic()
{
  if (disableTimer.Get() > 2.0)
  {
    IO.drivebase.SetCoast();
  }
  IO.RJV.Reset();
  IO.shooter.Stop();
  IO.shooter.IntakeRetract();
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
      IO.climber.ClimberRetract();
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
  // Shared Stuff
  double indexer = 0.0;
  double leftTrigOp = IO.ds.Operator.GetTriggerAxis(GenericHID::kLeftHand);
  double rightTrigOp = IO.ds.Operator.GetTriggerAxis(GenericHID::kRightHand);
  //double leftTrigDr = IO.ds.Driver.GetTriggerAxis(GenericHID::kLeftHand);
  double rightTrigDr = IO.ds.Driver.GetTriggerAxis(GenericHID::kRightHand);
  double intakeSpeed = leftTrigOp - rightTrigOp - rightTrigDr;
  intakeSpeed = Deadband(-intakeSpeed, deadband);

  //
  // Drive
  //
  double forward = Deadband(IO.ds.Driver.GetY(GenericHID::kLeftHand) * -1.0, deadband);
  double rotate = Deadband(IO.ds.Driver.GetX(GenericHID::kRightHand) * -1.0, deadband);
  if (abs(forward) < 0.05)
  {
    rotate = (CABICA * pow(rotate, 3)) + (CABICB * rotate);
  }
  else
  {
    rotate = (cubicA * pow(rotate, 3)) + (cubicB * rotate);
  }

  // Limit turning rate normally
  if (!IO.ds.Driver.GetStickButton(GenericHID::kRightHand))
  {
    rotate *= kDriveTurnLimit;
  }
  // TODO Cubic Smoothing

  //
  // Shooting Presets
  //
  if (IO.ds.Operator.GetDownButton())
  {
    IO.shooter.SetVelocity(2450.0);
    PresetShooterRPM = 2450.0;
    PresetHoodAngle = 23;
    PresetVisionPipeline = IO.RJV.Pipe::ThreeClose;
    liteOn = true;

    IO.RJV.SetPipeline(PresetVisionPipeline);
  }
  else if (IO.ds.Operator.GetUpButton())
  {
    IO.shooter.SetVelocity(4370.0);
    PresetShooterRPM = 4370.0;
    PresetHoodAngle = 66.6; //DUN DUN DUNNNN The long shot comes with a price...
    PresetVisionPipeline = IO.RJV.Pipe::LongShot;
    liteOn = true;

    IO.RJV.SetPipeline(PresetVisionPipeline);
  }
  else if (IO.ds.Operator.GetLeftButton())
  {
    IO.shooter.SetVelocity(3000.0);
    PresetShooterRPM = 3000.0;
    PresetHoodAngle = 60.5;
    PresetVisionPipeline = IO.RJV.Pipe::ThreeFar;
    liteOn = true;

    IO.RJV.SetPipeline(PresetVisionPipeline);
  }
  else if (IO.ds.Operator.GetRightButton())
  {
    IO.shooter.SetVelocity(3500.0);
    PresetShooterRPM = 3500.0;
    PresetHoodAngle = 64.5;
    PresetVisionPipeline = IO.RJV.Pipe::TwoClose;
    liteOn = true;

    IO.RJV.SetPipeline(PresetVisionPipeline);
  }
  else if (IO.ds.Operator.GetScreenShotButton())
  {
    PresetShooterRPM = PRESET_RPM;
    IO.shooter.SetVelocity(PresetShooterRPM);
    PresetHoodAngle = PRESET_HOOD;
    PresetVisionPipeline = IO.RJV.Pipe::TwoClose;
    liteOn = true;

    IO.RJV.SetPipeline(PresetVisionPipeline);
  }

  //
  // Vision Here
  //

  // Normal Driving
  if (IO.ds.Driver.GetSquareButton())
  {
    if (!visionTestOS)
    {
      visionTest.Reset();
      visionTest.Start();
      visionTestOS = true;
    }
    data = IO.RJV.Run(0);
    //data = IO.RJV.Run(PresetVisionPipeline);
    if (data.filled)
    {
      double visionTime = visionTest.Get();
      SmartDashboard::PutNumber("Vision Time", visionTime);
      if (IO.drivebase.VisionAim(forward, data.angle, 0.3))
      {
        PresetShooterRPM = PRESET_RPM;
        IO.shooter.SetVelocity(PresetShooterRPM);
        PresetHoodAngle = PRESET_HOOD;
        SmartDashboard::PutNumber("Vision Time", visionTime);
        visionTest.Stop();
      }
    }
    else
      IO.drivebase.Arcade(forward, rotate);
  }
  else if (IO.ds.Driver.GetCrossButton())
  {
    if (!visionTestOS)
    {
      visionTest.Reset();
      visionTest.Start();
      visionTestOS = true;
    }
    data = IO.RJV.Run(0);
    //data = IO.RJV.Run(PresetVisionPipeline);
    if (data.filled)
    {
      double visionTime = visionTest.Get();
      SmartDashboard::PutNumber("Vision Time", visionTime);
      if (IO.drivebase.VisionAim(forward, data.angle, 0.3))
      {
        SmartDashboard::PutNumber("Vision Time", visionTime);
        visionTest.Stop();
      }
    }
    else
      IO.drivebase.Arcade(forward, rotate);
  }
  else if (liteOn)
  {
    data = IO.RJV.Run(PresetVisionPipeline);
    IO.drivebase.Arcade(forward, rotate);
    data.filled = false;
    picCt = 0;
    tpCt = 0;
    visionTestOS = false;
  }
  else
  {
    IO.drivebase.Arcade(forward, rotate);
    data.filled = false;
    picCt = 0;
    tpCt = 0;
    IO.RJV.Reset();
    visionTestOS = false;
  }

  // Reset Vision

  //
  // Hood Control
  //
  double hoodAnalog = Deadband(IO.ds.Operator.GetY(GenericHID::kRightHand) * -1.0, deadband);
  if (hoodAnalog != 0.0 || PresetHoodAngle < 0.0)
  {
    PresetHoodAngle = -1.0;
    IO.shooter.SetHood(hoodAnalog);
    IO.shooter.ResetHood();
  }
  else if (PresetHoodAngle >= 0.0)
  {
    IO.shooter.SetHoodAngle(PresetHoodAngle);
  }
  else
  {
    IO.shooter.SetHood(0.0);
  }

  //
  // Manual Shooting System
  //
  atSpeed = ((abs(PresetShooterRPM - IO.shooter.GetVelocity()) < 150.0));
  atAngle = ((abs(PresetHoodAngle - IO.shooter.GetHoodAngle()) < 1.0));

  if ((IO.ds.Operator.GetTriangleButton() || (IO.ds.Driver.GetTriangleButton() && atSpeed && atAngle)) && abs(leftTrigOp == 0.0))
  {
    indexer = indexerSpeed;
    IO.shooter.SetFeeder(1.0);
  }
  else
  {
    IO.shooter.SetFeeder(0.0);
  }

  // Hood Lock
  //IO.shooter.SetHoodLock(IO.ds.Operator.GetTriangleButton() || IO.ds.Driver.GetTriangleButton());

  if (IO.ds.Operator.GetCircleButton())
  {
    IO.shooter.SetShooter(0.0);
    liteOn = false;
  }

  //
  // Intake
  //

  // Anti-jam
  if (abs(leftTrigOp) > 0.0)
  {
    intakeSpeed = 0.0;
    indexer = -indexerSpeed;
    IO.shooter.SetFeeder(-1.0);
  }
  else if (IO.ds.Operator.GetSquareButton())
  {
    intakeSpeed = -0.8;
  }
  else if (IO.ds.Operator.GetCrossButton())
  {
    intakeSpeed = -0.8;
    indexer = -indexerSpeed;
  }
  else if (intakeSpeed > 0.0)
  {
    intakeSpeed = 0.8;
    indexer = indexerSpeed;
  }

  IO.shooter.SetIntake(intakeSpeed);

  if (IO.ds.Operator.GetBumper(GenericHID::kLeftHand))
  {
    IO.shooter.IntakeRetract();
  }

  if (IO.ds.Operator.GetBumper(GenericHID::kRightHand))
  {
    liteOn = false;
    IO.shooter.IntakeDeploy();
    PresetHoodAngle = 0.0;
  }

  //
  // Indexer
  //
  IO.shooter.SetIndexer(indexer);

  //
  // Climber
  //
  double climbAnalog = Deadband(IO.ds.Operator.GetY(GenericHID::kLeftHand) * -1, deadband);
  IO.climber.SetClimber(climbAnalog);

  if (IO.ds.Operator.GetPSButton() || IO.ds.Driver.GetPSButton())
  {
    IO.climber.ClimberDeploy();
  }
  else
  {
    IO.climber.ClimberRetract();
  }

  //
  // ColorWheel
  //
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
  SmartDashboard::PutNumber("Cubic A", CABICA);
  SmartDashboard::PutNumber("Cubic B", CABICB);
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
    //SmartDashboard::PutNumber("RPM TOLERANCE", RPM_TOLERANCE);
    SmartDashboard::PutNumber("PRESET_HOOD", PRESET_HOOD);
    SmartDashboard::PutBoolean("AtSpeed", atSpeed);
    SmartDashboard::PutNumber("PRESET_RPM", PRESET_RPM);
    SmartDashboard::PutBoolean("AtAngle", atAngle);
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
