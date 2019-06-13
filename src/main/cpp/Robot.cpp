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
  bool btnPSDr = IO.ds.DriverPS.GetPSButton();
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

  // PS4 & XBox controller remapping
  double forward;
  double rotate;
  double strafe;
  double forwardR;
  double leftTrigDr;
  double rightTrigDr;
  bool leftBumpDr;
  bool rightBumpDr;
  bool btnACrossDr;
  bool btnYTriangleDr;
  bool btnBCircleDr;
  bool btnXSquareDr;
  bool btnUpDr;
  bool btnRightDr;
  bool btnLeftDr;
  bool btnDownDr;
  bool btnBackDr;
  bool btnStartDr;
  bool btnPSDr;

  // PS4 Controller (Operator)
  double leftTrigOp;
  double rightTrigOp; 
  double leftOpY;
  double rightOpY;
  bool leftBumpOp;
  bool rightBumpOp;
  bool btnACrossOp;
  bool btnBCircleOp;
  bool btnYTriangleOp;
  bool btnXSquareOp;
  bool btnUpOp;
  bool btnRightOp;
  bool btnLeftOp;
  bool btnDownOp;
  bool btnBackOp;
  bool btnStartOp;
  bool btnPSOp;

  // PS4 Controller (Driver)
  if (IO.ds.chooseController.GetSelected() == IO.ds.sPS4)
  {
  forward = IO.ds.DriverPS.GetY(GenericHID::kLeftHand) * -1;
  rotate = IO.ds.DriverPS.GetX(GenericHID::kRightHand) * -1;
  strafe = IO.ds.DriverPS.GetX(GenericHID::kLeftHand);
  forwardR = IO.ds.DriverPS.GetY(GenericHID::kRightHand) * -1;
  leftTrigDr = IO.ds.DriverPS.GetTriggerAxis(GenericHID::kLeftHand);
  rightTrigDr = IO.ds.DriverPS.GetTriggerAxis(GenericHID::kRightHand);
  leftBumpDr = IO.ds.DriverPS.GetBumper(GenericHID::kLeftHand);
  rightBumpDr = IO.ds.DriverPS.GetBumper(GenericHID::kRightHand);
  btnACrossDr = IO.ds.DriverPS.GetCrossButton();
  btnYTriangleDr = IO.ds.DriverPS.GetTriangleButtonPressed();
  btnBCircleDr = IO.ds.DriverPS.GetCircleButton();
  btnXSquareDr = IO.ds.DriverPS.GetSquareButton();
  btnUpDr = IO.ds.DriverPS.GetUpButton();
  btnRightDr = IO.ds.DriverPS.GetRightButton();
  btnLeftDr = IO.ds.DriverPS.GetLeftButton();
  btnDownDr = IO.ds.DriverPS.GetDownButton();
  btnBackDr = IO.ds.DriverPS.GetScreenShotButton();
  btnStartDr = IO.ds.DriverPS.GetOptionsButtonPressed();
  btnPSDr = IO.ds.DriverPS.GetPSButtonPressed();

  // PS4 Controller (Operator)
  leftTrigOp = IO.ds.OperatorPS.GetTriggerAxis(GenericHID::kLeftHand);
  rightTrigOp = IO.ds.OperatorPS.GetTriggerAxis(GenericHID::kRightHand); 
  leftOpY = IO.ds.OperatorPS.GetY(GenericHID::kLeftHand);
  rightOpY = IO.ds.OperatorPS.GetY(GenericHID::kRightHand);
  leftBumpOp = IO.ds.OperatorPS.GetBumper(GenericHID::kLeftHand);
  rightBumpOp = IO.ds.OperatorPS.GetBumper(GenericHID::kRightHand);
  btnACrossOp = IO.ds.OperatorPS.GetCrossButton();
  btnBCircleOp = IO.ds.OperatorPS.GetCircleButton();
  btnYTriangleOp = IO.ds.OperatorPS.GetTriangleButtonPressed();
  btnXSquareOp = IO.ds.OperatorPS.GetSquareButton();
  btnUpOp = IO.ds.OperatorPS.GetUpButton();
  btnRightOp = IO.ds.OperatorPS.GetRightButton();
  btnLeftOp = IO.ds.OperatorPS.GetLeftButton();
  btnDownOp = IO.ds.OperatorPS.GetDownButton();
  btnBackOp = IO.ds.OperatorPS.GetScreenShotButton();
  btnStartOp = IO.ds.OperatorPS.GetOptionsButton();
  btnPSOp = IO.ds.OperatorPS.GetPSButtonPressed();

  } else  {
    // Xbox Controller (Driver)
    forward = IO.ds.DriverXB.GetY(GenericHID::kLeftHand) * -1;
    rotate = IO.ds.DriverXB.GetX(GenericHID::kRightHand) * -1;
    strafe = IO.ds.DriverXB.GetX(GenericHID::kLeftHand);
    forwardR = IO.ds.DriverXB.GetY(GenericHID::kRightHand) * -1;
    leftTrigDr = IO.ds.DriverXB.GetTriggerAxis(GenericHID::kLeftHand);
    rightTrigDr = IO.ds.DriverXB.GetTriggerAxis(GenericHID::kRightHand); //Negative
    leftBumpDr = IO.ds.DriverXB.GetBumper(GenericHID::kLeftHand);
    rightBumpDr = IO.ds.DriverXB.GetBumper(GenericHID::kRightHand);
    btnACrossDr = IO.ds.DriverXB.GetAButton();
    btnBCircleDr = IO.ds.DriverXB.GetBButton();
    btnXSquareDr = IO.ds.DriverXB.GetXButton();
    btnYTriangleDr = IO.ds.DriverXB.GetYButtonPressed();
    btnUpDr = IO.ds.DriverXB.GetPOV() == 315 || IO.ds.DriverXB.GetPOV() == 0 || IO.ds.DriverXB.GetPOV() == 45;
    btnRightDr = IO.ds.DriverXB.GetPOV() == 45 || IO.ds.DriverXB.GetPOV() == 90 || IO.ds.DriverXB.GetPOV() == 135;
    btnLeftDr = IO.ds.DriverXB.GetPOV() == 225 || IO.ds.DriverXB.GetPOV() == 270 || IO.ds.DriverXB.GetPOV() == 315;
    btnDownDr = IO.ds.DriverXB.GetPOV() == 135 || IO.ds.DriverXB.GetPOV() == 180 || IO.ds.DriverXB.GetPOV() == 225;
    btnBackDr = IO.ds.DriverXB.GetBackButton();
    btnStartDr = IO.ds.DriverXB.GetStartButtonPressed();

    leftTrigOp = IO.ds.OperatorXB.GetTriggerAxis(GenericHID::kLeftHand);
    rightTrigOp = IO.ds.OperatorXB.GetTriggerAxis(GenericHID::kRightHand); //Negative
    leftOpY = IO.ds.OperatorXB.GetY(GenericHID::kLeftHand);
    rightOpY = IO.ds.OperatorXB.GetY(GenericHID::kRightHand);
    leftBumpOp = IO.ds.OperatorXB.GetBumper(GenericHID::kLeftHand);
    rightBumpOp = IO.ds.OperatorXB.GetBumper(GenericHID::kRightHand);
    btnYTriangleOp = IO.ds.OperatorXB.GetYButtonPressed();
    btnACrossOp = IO.ds.OperatorXB.GetAButton();
    btnBCircleOp = IO.ds.OperatorXB.GetBButton();
    btnXSquareOp = IO.ds.OperatorXB.GetXButton();
    btnUpOp = IO.ds.OperatorXB.GetPOV() == 315 || IO.ds.OperatorXB.GetPOV() == 0 || IO.ds.OperatorXB.GetPOV() == 45;
    btnRightOp = IO.ds.OperatorXB.GetPOV() == 45 || IO.ds.OperatorXB.GetPOV() == 90 || IO.ds.OperatorXB.GetPOV() == 135;
    btnLeftOp = IO.ds.OperatorXB.GetPOV() == 225 || IO.ds.OperatorXB.GetPOV() == 270 || IO.ds.OperatorXB.GetPOV() == 315;
    btnDownOp = IO.ds.OperatorXB.GetPOV() == 135 || IO.ds.OperatorXB.GetPOV() == 180 || IO.ds.OperatorXB.GetPOV() == 225;
    btnBackOp = IO.ds.OperatorXB.GetBackButton();
    btnStartOp = IO.ds.OperatorXB.GetStartButton();
  }

  // Drive
  forward = Deadband(forward, deadband);
  rotate = Deadband(rotate, deadband);
  forwardR = Deadband(forwardR, deadband);
  strafe = Deadband(strafe, deadband);

  
  

  if(driveMode == SplitArcade)
  {
    IO.drivebase.Arcade(forward, rotate);

    if(btnStartDr) driveMode = Tank;
  }
  else if(driveMode == Tank)
  {
    IO.drivebase.Tank(forward, forwardR);
    
    if(btnStartDr) driveMode = Mecanum;
  }
  else if(driveMode == Mecanum)
  {
    IO.drivebase.Mecanum(forward, rotate, strafe);
    
    if(btnStartDr) driveMode = SplitArcade;
  }
  
  if(leftBumpDr) 
  {
    IO.drivebase.SetLowGear();
  }

  if(rightBumpDr) 
  {
    IO.drivebase.SetHighGear();
  }

  // Manip
  IO.manip.SetA(leftTrigDr - rightTrigDr + leftTrigOp - rightTrigOp  );

  IO.manip.SetSol1(btnXSquareDr | btnXSquareOp);

  if(btnYTriangleDr | btnYTriangleOp)
  {
    IO.manip.ToggleSol2();
  }

  if(btnACrossDr | btnACrossOp)
  {
    IO.manip.SetB(1.0);
  }

  if(btnBCircleDr | btnBCircleOp)
  {
    IO.manip.SetB(0.0);
  }

}
void Robot::TestPeriodic() {}

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
  //  Don't update smart dash every loop,
  // it causes watchdog warnings
  if (smartDashSkip > 30)
  {
    smartDashSkip = 0;
  }

  //
  // Sensor Override
  //
  switch (smartDashSkip)
  {
  case 0:
  {
    if (IO.ds.chooseDriveLimit.GetSelected() == IO.ds.sUnlimitted)
    {
      IO.drivebase.ActivateSensorOverride();
    }
    else
    {
      IO.drivebase.DeactivateSensorOverride();
    }

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



  default:
  {
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
