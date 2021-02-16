/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "auto/AutoPrograms.hpp"
#include <string>
#include "robotmap.hpp"
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <opencv2/videoio.hpp>
#include <frc/PowerDistributionPanel.h>
#include <frc/Timer.h>

using namespace cv;

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

private:
  double Deadband(double input, double deadband);
  void UpdateSD();

  robotmap IO;
  AutoPrograms autoPrograms{IO};
  LiveWindow &m_lw = *frc::LiveWindow::GetInstance();

  const double deadband = 0.05;
  const double cubicA = 0.516;
  const double cubicB = 0.316;

  double kDriveTurnLimit = 1.0;

  double indexerSpeed = 1.0;

  // Dont update smart dash every loop
  int smartDashSkip = 0;

  PowerDistributionPanel *pdp = new PowerDistributionPanel();

  // Vision Stuff
  vision::RJVisionPipeline::visionData data;
  int tpCt = 0;
  int picCt = 0;
  bool liteOn = false;
  
  // Presets
  double PresetShooterRPM = 0.0;
  double PresetHoodAngle = -1.0;
  int PresetVisionPipeline = 0;
  //const double rpmTolerance = 60.0;

  bool atSpeed = false;
  bool atAngle = false;

  Timer disableTimer;

  Timer visionTest;
  bool visionTestOS = false;

  bool feedbackMode = false;
  bool feedbackModeOS = false;
  bool runAuto = false;

#define PRESET_RPM (SmartDashboard::GetNumber("PRESET_RPM", 2000))
#define PRESET_HOOD (SmartDashboard::GetNumber("PRESET_HOOD", 40))

#define CABICA (SmartDashboard::GetNumber("Cubic A", 0.2244))
#define CABICB (SmartDashboard::GetNumber("Cubic B", 0.2131))
//#define RPM_TOLERANCE (SmartDashboard::GetNumber("RPM TOLERANCE", 40.0))

  // Test Mode
  SendableChooser<std::string> chooseTestDevice;
  const std::string sDrive = "1 - Drive";
  const std::string sIntake = "2 - Intake";
  const std::string sIndexer = "3 - Indexer";
  const std::string sFeeder = "4 - Feeder";
  const std::string sShooter = "5.0 - Shooter";
  const std::string sShooterVelocity = "5.1 - ShooterVelocity";
  const std::string sHood = "6 - Hood";
  const std::string sClimber = "7 - Climber";
  const std::string sColorWheel = "8 - Color Wheel";
  const std::string sVision = "9 - Vision";
};
