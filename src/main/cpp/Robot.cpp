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
}

void Robot::RobotPeriodic()
{
  // Reset Encoders
  bool btnPSDr = IO.ds.Driver.GetPSButton();
  if (btnPSDr)
  {
    IO.drivebase.ResetEncoders();
    autoPrograms.Init();
  }

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
}

void Robot::DisabledInit()
{
}

void Robot::TeleopPeriodic()
{

  // Drive Mode
  if (IO.ds.Driver.GetOptionsButtonPressed())
  {
    if (driveMode == SplitArcade)
    {
      driveMode = Tank;
    }
    else if (driveMode == Tank)
    {
      driveMode = Mecanum;
    }
    else if (driveMode == Mecanum)
    {
      driveMode = SplitArcade;
    }
  }

  // Drive
  double forward = Deadband(IO.ds.Driver.GetY(GenericHID::kLeftHand) * -1, deadband);
  double rotate = Deadband(IO.ds.Driver.GetX(GenericHID::kRightHand) * -1, deadband);
  double forwardR = Deadband(IO.ds.Driver.GetY(GenericHID::kRightHand) * -1, deadband);
  double strafe = Deadband(IO.ds.Driver.GetX(GenericHID::kLeftHand), deadband);

  if (driveMode == SplitArcade)
  {
    IO.drivebase.Arcade(forward, rotate);
  }
  else if (driveMode == Tank)
  {
    IO.drivebase.Tank(forward, forwardR);
  }
  else if (driveMode == Mecanum)
  {
    IO.drivebase.Mecanum(forward, rotate, strafe);
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

  // Manip
  double leftTrigDr = IO.ds.Driver.GetTriggerAxis(GenericHID::kLeftHand);
  double rightTrigDr = IO.ds.Driver.GetTriggerAxis(GenericHID::kRightHand);
  double leftTrigOp = IO.ds.Operator.GetTriggerAxis(GenericHID::kLeftHand);
  double rightTrigOp = IO.ds.Operator.GetTriggerAxis(GenericHID::kRightHand);
  //IO.manip.SetA(leftTrigDr - rightTrigDr + leftTrigOp - rightTrigOp);

  IO.manip.SetSol1(IO.ds.Driver.GetSquareButton() | IO.ds.Operator.GetSquareButton());

  if (IO.ds.Driver.GetTriangleButton() | IO.ds.Operator.GetTriangleButton())
  {
    IO.manip.ToggleSol2();
  }

  if (IO.ds.Driver.GetCrossButton() | IO.ds.Operator.GetCrossButton())
  {
    IO.manip.SetA(1.0);
    IO.manip.SetB(1.0);
    IO.manip.SetC(1.0);
    IO.manip.SetD(1.0);
  }

  if ( IO.ds.Driver.GetCircleButton() |  IO.ds.Operator.GetCircleButton())
  {
    IO.manip.SetA(0.0);
    IO.manip.SetB(0.0);
    IO.manip.SetC(0.0);
    IO.manip.SetD(0.0);
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
  case 0:
  {
    IO.drivebase.SensorOverride( IO.ds.chooseDriveLimit.GetSelected() == IO.ds.sUnlimitted );
    break;
  }

  case 2:
  {
    IO.drivebase.UpdateSmartdash();

    SmartDashboard::PutNumber("DriveMode", driveMode);
    break;
  }

  case 3:
  {
    IO.manip.UpdateSmartdash();
    break;
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
