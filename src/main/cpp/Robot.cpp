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
#include <frc/smartdashboard/SendableChooser.h>
#include "Drivetrain.h"
#include <frc/Preferences.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>
frc::Preferences *prefs;
frc::SendableChooser<std::string> m_chooser;
class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override {
   SetNetworkTablesFlushEnabled(true);
    m_chooser.SetDefaultOption("None", "None");
    m_chooser.AddOption("TEST", "TEST");
    m_chooser.AddOption("Salom", "Salom");
    m_chooser.AddOption("Barrel", "Barrel");
    m_chooser.AddOption("Bounce", "Bounce");
    m_chooser.AddOption("A-Red(PathWeaver)","A-Red(PathWeaver)");
    m_chooser.AddOption("A-Blue(PathWeaver)","A-Blue(PathWeaver)");
    m_chooser.AddOption("B-Red(PathWeaver)","B-Red(PathWeaver)");
    m_chooser.AddOption("B-Blue(PathWeaver)","B-Blue(PathWeaver)");
    frc::SmartDashboard::PutData(&m_chooser);
  }

  void RobotPeriodic() override { m_drive.Periodic(); }

  void AutonomousInit() override {
     auto path = m_chooser.GetSelected();
     if(m_chooser.GetSelected() == "None") return;
      if(path == "A-Red(PathWeaver)")
      {
      wpi::SmallString<64> deployDirectory;
      frc::filesystem::GetDeployDirectory(deployDirectory);
      wpi::sys::path::append(deployDirectory, "output");
      wpi::sys::path::append(deployDirectory, "GS_A_Red.wpilib.json");
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
    }
    else if(path == "A-Blue(PathWeaver)")
      {
      wpi::SmallString<64> deployDirectory;
      frc::filesystem::GetDeployDirectory(deployDirectory);
      wpi::sys::path::append(deployDirectory, "output");
      wpi::sys::path::append(deployDirectory, "GS_A_Blue.wpilib.json");
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
    }
    
    else if(path == "B-Red(PathWeaver)")  
   {
      wpi::SmallString<64> deployDirectory;
      frc::filesystem::GetDeployDirectory(deployDirectory);
      wpi::sys::path::append(deployDirectory, "output");
      wpi::sys::path::append(deployDirectory, "GS_B_Red.wpilib.json");
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
   }
     else if(path == "B-Blue(PathWeaver)")
     {
      wpi::SmallString<64> deployDirectory;
      frc::filesystem::GetDeployDirectory(deployDirectory);
      wpi::sys::path::append(deployDirectory, "output");
      wpi::sys::path::append(deployDirectory, "GS_B_Blue.wpilib.json");
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
     }
      else if(path == "Salom")
      {
      wpi::SmallString<64> deployDirectory;
      frc::filesystem::GetDeployDirectory(deployDirectory);
      wpi::sys::path::append(deployDirectory, "output");
      wpi::sys::path::append(deployDirectory, "Salom.wpilib.json");
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
    }
     else if(path == "Barrel")
      {
      wpi::SmallString<64> deployDirectory;
      frc::filesystem::GetDeployDirectory(deployDirectory);
      wpi::sys::path::append(deployDirectory, "output");
      wpi::sys::path::append(deployDirectory, "Barrel.wpilib.json");
      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
    }
    else if(path == "Bounce")
      {
       wpi::SmallString<64> deployDirectory;
      frc::filesystem::GetDeployDirectory(deployDirectory);
      wpi::sys::path::append(deployDirectory, "output");
      wpi::sys::path::append(deployDirectory, "Bounce#.wpilib.json");
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
      auto start_time = s1.back().t + 10_ms;
      for (size_t i = 0; i < s2.size(); i++)
      {
        s2[i].t += start_time;         
      } 
      auto start_time2 = s2.back().t + 10_ms;
      for (size_t i = 0; i < s3.size(); i++)
      {
        s3[i].t += start_time2;
      }
       auto start_time3 = s3.back().t + 10_ms;
      for (size_t i = 0; i < s4.size(); i++)
      {
        s4[i].t += start_time3;
      }
      // Add the second path to the overall path
      s1.insert( s1.end(), s2.begin(), s2.end() );
      s1.insert( s1.end(), s3.begin(), s3.end() );
      s1.insert( s1.end(), s4.begin(), s4.end() );
      // Save this Trajectory
      m_trajectory = frc::Trajectory(s1);
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
    auto path = m_chooser.GetSelected();
     if(m_chooser.GetSelected() == "None") return;
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
  frc::Trajectory m_trajectory;
  frc::Trajectory m_trajectoryB;
    frc::Trajectory m_trajectoryC;
    frc::Trajectory m_trajectoryD;
  frc::RamseteController m_ramsete;
  frc2::Timer m_timer;
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
