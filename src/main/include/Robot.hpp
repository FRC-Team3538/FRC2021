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
  void TestInit() override;
  void TestPeriodic() override;

private:
  robotmap IO;
  AutoPrograms autoPrograms{IO};
  LiveWindow &m_lw = *frc::LiveWindow::GetInstance();

  const double deadband = 0.1;

  double indexerSpeed = 1.0; // TODO SmartDash input
  double kDriveTurnLimit = 0.65;

  int tpCt = 0;
  double df;
  bool blastOS = false;
  double c;

  // Dont update smart dash every loop
  int smartDashSkip = 0;

  double Deadband(double input, double deadband);
  void UpdateSD();

  PowerDistributionPanel *pdp = new PowerDistributionPanel();

  // Vision Stuff
  vision::RJVisionPipeline::visionData data;

  // Timer
  Timer manualShootTimer;

  // Test Mode
  SendableChooser<std::string> chooseTestDevice;
  const std::string sIntake = "0 - Intake";
  const std::string sIndexer = "1 - Indexer";
  const std::string sFeeder = "2 - Feeder";
  const std::string sShooter = "3 - Shooter";
  const std::string sHood = "4 - Hood";
  const std::string sClimber = "5 - Climber";
  const std::string sColorWheel = "6 - Color Wheel";
};
