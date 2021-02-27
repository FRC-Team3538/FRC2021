/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/SlewRateLimiter.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/controller/RamseteController.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/Timer.h>
#include <frc/Preferences.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "Drivetrain.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override {
   // Flush NetworkTables every loop. This ensures that robot pose and other
    // values are sent during every iteration.
    SetNetworkTablesFlushEnabled(true);
    m_chooser.SetDefaultOption("None", "None");
    m_chooser.AddOption("TEST", "TEST");
    m_chooser.AddOption("A-Red",  "A-Red");
    m_chooser.AddOption("A-Blue", "A-Blue");
    m_chooser.AddOption("B-Red",  "B-Red");
    m_chooser.AddOption("B-Blue", "B-Blue");
    m_chooser.AddOption("Barrel", "Barrel");
    m_chooser.AddOption("Bounce", "Bounce");
    m_chooser.AddOption("Salom",  "Salom");
    frc::SmartDashboard::PutData(&m_chooser);
  }  

  void RobotPeriodic() override { m_drive.Periodic(); }

  void AutonomousInit() override {
    auto path = m_chooser.GetSelected();
    if(path == "A-Red")
    {
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d(40_in, 120_in, -35_deg), {
            frc::Translation2d(150_in, 60_in), 
            frc::Translation2d(180_in, 150_in)
          }, 
          frc::Pose2d(360_in, 170_in, 0_deg),
          frc::TrajectoryConfig(3_mps, 3_mps_sq)
      );
    }
    else if(path == "A-Blue")
    {
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(35_in, 30_in, 0_deg), 
        {
          frc::Translation2d(180_in, 30_in), 
          frc::Translation2d(210_in, 120_in),
          frc::Translation2d(278_in, 100_in)
        }, 
        frc::Pose2d(330_in, 60_in, 0_deg),
        frc::TrajectoryConfig(3_mps, 3_mps_sq));
    }
    else if(path == "B-Red")
    {
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(40_in, 120_in, 0_deg),
        {
          frc::Translation2d(90_in, 120_in), 
          frc::Translation2d(150_in, 60_in), 
          frc::Translation2d(210_in, 120_in)
        },
        frc::Pose2d(330_in, 120_in, 0_deg), 
        frc::TrajectoryConfig(3_mps, 3_mps_sq)); 
    }
    else if(path == "B-Blue")
    {
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(40_in, 60_in, 0_deg),
        {
          frc::Translation2d(180_in, 60_in),
          frc::Translation2d(240_in, 120_in),
          frc::Translation2d(300_in, 60_in)
        },
        frc::Pose2d(330_in, 60_in, 0_deg),
        frc::TrajectoryConfig(3_mps, 3_mps_sq));
    }
     else if(path == "Barrel")
    {
      wpi::SmallString<64> deployDirectory;
      frc::filesystem::GetDeployDirectory(deployDirectory);
      wpi::sys::path::append(deployDirectory, "output");
      wpi::sys::path::append(deployDirectory, "Barrel path.wpilib.json");
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
    }
    else if(path == "Bounce")
    {
      wpi::SmallString<64> deployDirectory;
      frc::filesystem::GetDeployDirectory(deployDirectory);
      wpi::sys::path::append(deployDirectory, "output");
      wpi::sys::path::append(deployDirectory, "bounce path.wpilib.json");
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
    }
    else if(path == "Salom")
    {
      wpi::SmallString<64> deployDirectory;
      frc::filesystem::GetDeployDirectory(deployDirectory);
      wpi::sys::path::append(deployDirectory, "output");
      wpi::sys::path::append(deployDirectory, "Salom Path.wpilib.json");
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
    }
    else if(path == "TEST")
    {
      // Get path from preferences for testing & tuning
      prefs = frc::Preferences::GetInstance();
      auto x0 = units::inch_t(prefs->GetDouble("x0", 0));
      auto y0 = units::inch_t(prefs->GetDouble("y0", 0));
      auto r0 = units::degree_t(prefs->GetDouble("r0", 0));
      auto x1 = units::inch_t(prefs->GetDouble("x1", 0));
      auto y1 = units::inch_t(prefs->GetDouble("y1", 0));
      auto x2 = units::inch_t(prefs->GetDouble("x2", 0));
      auto y2 = units::inch_t(prefs->GetDouble("y2", 0));
      auto x3 = units::inch_t(prefs->GetDouble("x3", 0));
      auto y3 = units::inch_t(prefs->GetDouble("y3", 0));
      auto r3 = units::degree_t(prefs->GetDouble("r3", 0));
      auto vel = units::meters_per_second_t(prefs->GetDouble("v", 2));
      auto accel = units::meters_per_second_squared_t(prefs->GetDouble("a", 2));
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d(x0, y0, r0), {
            frc::Translation2d(x1, y1), 
            frc::Translation2d(x2, y2)
          }, 
          frc::Pose2d(x3, y3, r3),
          frc::TrajectoryConfig(vel, accel)
      );
    }
    else
    {
      // Default should not move...
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d(0_in, 0_in, 0_deg), {}, 
          frc::Pose2d(1_in, 0_in, 0_deg),
          frc::TrajectoryConfig(0.01_mps, 0.01_mps_sq)
      );
    }
   
    m_timer.Reset();
    m_timer.Start();
    m_drive.ResetOdometry(m_trajectory.InitialPose());
  }

  void AutonomousPeriodic() override {
    auto elapsed = m_timer.Get();
    auto reference = m_trajectory.Sample(elapsed);
    auto speeds = m_ramsete.Calculate(m_drive.GetPose(), reference);
    m_drive.Drive(speeds.vx, speeds.omega);
  }

  void TeleopPeriodic() override {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = -m_speedLimiter.Calculate(
                            m_controller.GetRawAxis(1)) *
                        Drivetrain::kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    auto rot = -m_rotLimiter.Calculate(
                   m_controller.GetRawAxis(2)) *
               Drivetrain::kMaxAngularSpeed;

    m_drive.Drive(xSpeed, rot);
  }

  void SimulationPeriodic() override { m_drive.SimulationPeriodic(); }

 private:
  frc::XboxController m_controller{0};

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_speedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  Drivetrain m_drive;
  frc::SendableChooser<std::string> m_chooser;
  frc::Trajectory m_trajectory;
  frc::RamseteController m_ramsete;
  frc2::Timer m_timer;
  frc::Preferences *prefs;
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
