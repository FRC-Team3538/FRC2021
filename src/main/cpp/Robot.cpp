// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/SlewRateLimiter.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/controller/RamseteController.h>
#include <frc/Preferences.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/Timer.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>

#include "Drivetrain.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override {
    // Flush NetworkTables every loop. This ensures that robot pose and other
    // values are sent during every iteration.
    SetNetworkTablesFlushEnabled(true);

    m_chooser.SetDefaultOption("None", "None");
    m_chooser.AddOption("TEST", "TEST");
    m_chooser.AddOption("A-Red", "A-Red");
    m_chooser.AddOption("A-Blue", "A-Blue");
    m_chooser.AddOption("B-Red", "B-Red");
    m_chooser.AddOption("B-Blue", "B-Blue");
    m_chooser.AddOption("Barrel", "Barrel");
    m_chooser.AddOption("Slalom", "Slalom");
    m_chooser.AddOption("Bounce", "Bounce");
    m_chooser.AddOption("PowerPort", "PowerPort");
    frc::SmartDashboard::PutData(&m_chooser);
  }

  void RobotPeriodic() override { 
    m_drive.Periodic(); 
  }

  void AutonomousInit() override {

    auto path = m_chooser.GetSelected();
    if(m_chooser.GetSelected() == "None") return;

    if(path == "A-Red")
    {
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d(40_in, 120_in, -35_deg), {
            frc::Translation2d(150_in, 60_in), 
            frc::Translation2d(180_in, 150_in)
          }, 
          frc::Pose2d(360_in, 170_in, 0_deg),
          frc::TrajectoryConfig(10_fps, 10_fps_sq)
      );
    }
    else if(path == "A-Blue")
    {
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d(45_in, 35_in, 0_deg), {
            frc::Translation2d(170_in, 35_in), 
            frc::Translation2d(210_in, 110_in)
          }, 
          frc::Pose2d(360_in, 70_in, 0_deg),
          frc::TrajectoryConfig(10_fps, 10_fps_sq)
      );
    }
    else if(path == "B-Red")
    {
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d(45_in, 150_in, -45_deg), {
            frc::Translation2d(150_in, 70_in), 
            frc::Translation2d(210_in, 110_in)
          }, 
          frc::Pose2d(360_in, 160_in, 0_deg),
          frc::TrajectoryConfig(10_fps, 10_fps_sq)
      );
    }
    else if(path == "B-Blue")
    {
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d(45_in, 60_in, 0_deg), {
            frc::Translation2d(170_in, 60_in), 
            frc::Translation2d(240_in, 110_in)
          }, 
          frc::Pose2d(360_in, 20_in, -35_deg),
          frc::TrajectoryConfig(10_fps, 10_fps_sq)
      );
    }
    else if(path == "Barrel")
    {
      wpi::SmallString<64> deployDirectory;
      frc::filesystem::GetDeployDirectory(deployDirectory);
      wpi::sys::path::append(deployDirectory, "output");
      wpi::sys::path::append(deployDirectory, "Barrel.wpilib.json");

      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
    }
    else if(path == "Slalom")
    {
      wpi::SmallString<64> deployDirectory;
      frc::filesystem::GetDeployDirectory(deployDirectory);
      wpi::sys::path::append(deployDirectory, "output");
      wpi::sys::path::append(deployDirectory, "Slalom.wpilib.json");

      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
    }
    else if(path == "Bounce")
    {
      wpi::SmallString<64> deployDirectory;
      frc::filesystem::GetDeployDirectory(deployDirectory);
      wpi::sys::path::append(deployDirectory, "output");
      wpi::sys::path::append(deployDirectory, "Bounce-#.wpilib.json");
      auto i = deployDirectory.rfind('#');

      // Load each path
      deployDirectory[i] = '1';
      auto t1 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
      auto s1 = t1.States();

      deployDirectory[i] = '2';
      auto t2 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
      auto s2 = t2.States();

      deployDirectory[i] = '3';
      auto t3 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
      auto s3 = t3.States();

      deployDirectory[i] = '4';
      auto t4 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
      auto s4 = t4.States();

      // Modify the time for each point in the second path
      {
        auto start_time = s1.back().t + 10_ms;
        for (size_t i = 0; i < s2.size(); i++)
        {
          s2[i].t += start_time;         
        } 

        // Add the second path to the overall path
        s1.insert( s1.end(), s2.begin(), s2.end() );
      }

      {
        auto start_time = s1.back().t + 10_ms;
        for (size_t i = 0; i < s3.size(); i++)
        {
          s3[i].t += start_time;         
        } 

        // Add the second path to the overall path
        s1.insert( s1.end(), s3.begin(), s3.end() );
      }

      {
        auto start_time = s1.back().t + 10_ms;
        for (size_t i = 0; i < s4.size(); i++)
        {
          s4[i].t += start_time;         
        } 

        // Add the second path to the overall path
        s1.insert( s1.end(), s4.begin(), s4.end() );
      }

      // Save this Trajectory
      m_trajectory = frc::Trajectory(s1);
    }
    else if(path == "PowerPort")
    {
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d(260_in, 90_in, 180_deg), { }, 
          frc::Pose2d(120_in, 90_in, 180_deg),
          frc::TrajectoryConfig(10_fps, 10_fps_sq)
      );
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

    m_timer.Reset();
    m_timer.Start();
    m_drive.ResetOdometry(m_trajectory.InitialPose());
  }

  void AutonomousPeriodic() override {
    auto path = m_chooser.GetSelected();
    if(m_chooser.GetSelected() == "None") return;

    auto elapsed = m_timer.Get();
    auto reference = m_trajectory.Sample(elapsed);
    auto speeds = m_ramsete.Calculate(m_drive.GetPose(), reference);
    m_drive.Drive(speeds.vx, speeds.omega);

  }

  void TeleopPeriodic() override {
    const auto xSpeed = -m_speedLimiter.Calculate( deadband(m_controller.GetRawAxis(1))) *
                        Drivetrain::kMaxSpeed;

    auto rot = -m_rotLimiter.Calculate(
                   deadband(m_controller.GetRawAxis(3))) *
               Drivetrain::kMaxAngularSpeed;

    m_drive.Drive(xSpeed, rot);
  }

  void DisabledPeriodic() override {
    m_drive.Drive(0_mps, 0_deg_per_s);
  }

  double deadband(double in){
    if(fabs(in) < 0.05) return 0.0;
    return in;
  }

  void SimulationPeriodic() override { m_drive.SimulationPeriodic(); }

 private:
  frc::XboxController m_controller{0};

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_speedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  Drivetrain m_drive{IsSimulation()};
  
  frc::Trajectory m_trajectory;
  frc::RamseteController m_ramsete;
  frc2::Timer m_timer;

  frc::Preferences *prefs;

  frc::SendableChooser<std::string> m_chooser;

};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
