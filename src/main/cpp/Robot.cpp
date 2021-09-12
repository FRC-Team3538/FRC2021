// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>

#include <frc/SlewRateLimiter.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/Preferences.h>
#include <frc/Filesystem.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>
#include <wpi/json.h>

#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/SwerveModule.h"
#include "subsystems/Shooter.h"
#include "lib/UniversalController.hpp"
#include <cmath>

#include <memory>
#include <thread>
#include <iostream>
#include "lib/csv.h"

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override
  {
    // Flush NetworkTables every loop.
    // This is probably not desireable for comp code, but good for debugging
    SetNetworkTablesFlushEnabled(false);
    frc::LiveWindow::GetInstance()->DisableAllTelemetry();
    frc::LiveWindow::GetInstance()->SetEnabled(false);

    // Controller Type Selection
    m_chooserControllerType.SetDefaultOption(kControllerTypePS4, frc::UniversalController::ControllerType::kPS4);
    m_chooserControllerType.AddOption(kControllerTypeXbox, frc::UniversalController::ControllerType::kXbox);
    m_chooserControllerType.AddOption(kControllerTypeStadia, frc::UniversalController::ControllerType::kStadia);
    frc::SmartDashboard::PutData(&m_chooserControllerType);

    m_chooserOperatorType.SetDefaultOption(kControllerTypePS4, frc::UniversalController::ControllerType::kPS4);
    m_chooserOperatorType.AddOption(kControllerTypeXbox, frc::UniversalController::ControllerType::kXbox);
    m_chooserOperatorType.AddOption(kControllerTypeStadia, frc::UniversalController::ControllerType::kStadia);
    frc::SmartDashboard::PutData(&m_chooserOperatorType);

    // Auto Program Selection
    m_chooser.SetDefaultOption(kAutoNone, kAutoNone);
    m_chooser.AddOption(kAutoBounce, kAutoBounce);

    m_chooser.AddOption(kAutoConstant, kAutoConstant);
    m_chooser.AddOption(kAutoToggle, kAutoToggle);
    m_chooser.AddOption(kAutoSweep, kAutoSweep);
    m_chooser.AddOption(kAutoDash, kAutoDash);
    frc::SmartDashboard::PutData(&m_chooser);

    // Smartdash
    frc::SmartDashboard::PutData("GamepadDriver", &m_controller);
    frc::SmartDashboard::PutData("GamepadOperator", &m_operator);
    frc::SmartDashboard::PutData("DriveBase", &m_swerve);
    frc::SmartDashboard::PutData("Shooter", &m_shooter);

    frc::SmartDashboard::SetDefaultNumber("auto/A_X", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/A_Y", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/A_R", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/B_X", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/B_Y", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/B_R", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/T", 4.0);
  }

  void DisabledInit() override
  {
    // Mostly for sim
    m_swerve.Drive(0_mps, 0_mps, 0_deg_per_s, false);
  }

  void RobotPeriodic() override
  {
    // PS4 | xbox | stadia controller mapping
    m_controller.SetControllerType(m_chooserControllerType.GetSelected());
    m_operator.SetControllerType(m_chooserOperatorType.GetSelected());

    // Update Drivebase Odometry
    m_swerve.UpdateOdometry();

    // Toggle Drive mode (Field | Robot Relative)
    if (m_controller.IsConnected())
    {
      if (m_controller.GetOptionsButtonPressed())
      {
        m_fieldRelative = !m_fieldRelative;
        if (m_fieldRelative)
        {
          std::cout << "Switching to Field Relative Control" << std::endl;
        }
        else
        {
          std::cout << "Switching to Normal Control" << std::endl;
        }
      }

      // Toggle Drive mode (Field | Robot Relative)
      if (m_controller.GetShareButtonPressed())
      {
        m_swerve.ResetYaw();
        std::cout << "Reset Gyro" << std::endl;
      }
    }

    m_shooter.Periodic();
  }

  void AutonomousInit() override
  {
    m_autoTimer.Reset();
    m_autoTimer.Start();

    // Get the location of the deploy directory where paths are stored
    wpi::SmallString<256> deploy_dir_ss;
    frc::filesystem::GetDeployDirectory(deploy_dir_ss);
    auto deploy_dir = std::string(deploy_dir_ss.c_str());

    //
    // Program Selection
    //auto deploy_dir = 
    auto program = m_chooser.GetSelected();
    if (program == kAutoDash)
    {
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d(30_in, 90_in, 0_deg),
          {// frc::Translation2d(150_in, 60_in),
           frc::Translation2d(180_in, 150_in)},
          frc::Pose2d(330_in, 90_in, 0_deg),
          frc::TrajectoryConfig(5_fps, 10_fps_sq));

      m_trajectory = m_trajectory.TransformBy({{0_m, 2_m}, 0_deg});

      // Display
      m_swerve.ShowTrajectory(m_trajectory);

      // Set initial pose of robot
      m_swerve.ResetYaw();
      m_swerve.ResetOdometry(m_trajectory.InitialPose());
    }
    else if (program == kAutoBounce)
    {
      frc::TrajectoryConfig config(15_fps, 20_fps_sq);
      config.AddConstraint(frc::CentripetalAccelerationConstraint{8_mps_sq});

      auto t1 = LoadWaypointCSV(deploy_dir + "/PathWeaver/Paths/Bounce-1.csv", config);
      auto s1 = t1.States();

      config.SetReversed(true);
      auto t2 = LoadWaypointCSV(deploy_dir + "/PathWeaver/Paths/Bounce-2.csv", config);
      auto s2 = t2.States();

      config.SetReversed(false);
      auto t3 = LoadWaypointCSV(deploy_dir + "/PathWeaver/Paths/Bounce-3.csv", config);
      auto s3 = t3.States();

      config.SetReversed(true);
      config.SetEndVelocity(10_fps);
      auto t4 = LoadWaypointCSV(deploy_dir + "/PathWeaver/Paths/Bounce-4.csv", config);
      auto s4 = t4.States();

      {
        auto start_time = s1.back().t + 10_ms;
        for (size_t i = 0; i < s2.size(); i++)
        {
          s2[i].t += start_time;
        }

        // Add the second path to the overall path
        s1.insert(s1.end(), s2.begin(), s2.end());
      }

      {
        auto start_time = s1.back().t + 10_ms;
        for (size_t i = 0; i < s3.size(); i++)
        {
          s3[i].t += start_time;
        }

        // Add the second path to the overall path
        s1.insert(s1.end(), s3.begin(), s3.end());
      }

      {
        auto start_time = s1.back().t + 10_ms;
        for (size_t i = 0; i < s4.size(); i++)
        {
          s4[i].t += start_time;
        }

        // Add the second path to the overall path
        s1.insert(s1.end(), s4.begin(), s4.end());
      }

      m_trajectory = frc::Trajectory(s1);

      // Display
      m_swerve.ShowTrajectory(m_trajectory);

      // Set initial pose of robot
      m_swerve.ResetYaw();
      auto p = frc::Pose2d{m_trajectory.InitialPose().Translation(), 0_deg};
      m_swerve.ResetOdometry(p);
    }

    std::cout << "Expected Time for this Path: " << m_trajectory.TotalTime() << std::endl;
  }

  void AutonomousPeriodic() override
  {
    // Shared Smartdash Inputs
    auto x1 = units::meters_per_second_t(frc::SmartDashboard::GetNumber("auto/A_X", 0.0));
    auto y1 = units::meters_per_second_t(frc::SmartDashboard::GetNumber("auto/A_Y", 0.0));
    auto r1 = units::degrees_per_second_t(frc::SmartDashboard::GetNumber("auto/A_R", 0.0));

    auto x2 = units::meters_per_second_t(frc::SmartDashboard::GetNumber("auto/B_X", 0.0));
    auto y2 = units::meters_per_second_t(frc::SmartDashboard::GetNumber("auto/B_Y", 0.0));
    auto r2 = units::degrees_per_second_t(frc::SmartDashboard::GetNumber("auto/B_R", 0.0));

    auto t = units::second_t(frc::SmartDashboard::GetNumber("auto/T", 4.0));
    auto autoTime = units::second_t(m_autoTimer.Get());

    //
    // Program Selection
    //
    auto program = m_chooser.GetSelected();
    if (program == kAutoNone)
    {
      return;
    }
    else if (program == kAutoDash ||
            program == kAutoBounce)
    {
      auto time = units::second_t(m_autoTimer.Get());
      auto state = m_trajectory.Sample(time);
      m_swerve.Drive(state, 0_deg); // state.pose.Rotation().Degrees());
    }
    else if (program == kAutoConstant)
    {
      // Just go at a fixed command
      m_swerve.Drive(x1, y1, r1, m_fieldRelative);
    }
    else if (program == kAutoToggle)
    {
      // Toggle between two setpoints
      if (autoTime > t / 2)
      {
        m_swerve.Drive(x1, y1, r1, m_fieldRelative);
      }
      else
      {
        m_swerve.Drive(x2, y2, r2, m_fieldRelative);
      }

      // Loop
      if (autoTime >= t)
        m_autoTimer.Reset();
    }
    else if (program == kAutoSweep)
    {
      // Protect div0
      if (t <= 0.0_s)
        return;

      auto a = units::math::abs(autoTime / t - 0.5) * 2.0;
      auto x = a * x1 + (1.0 - a) * x2;
      auto y = a * y1 + (1.0 - a) * y2;
      auto r = a * r1 + (1.0 - a) * r2;

      m_swerve.Drive(x, y, r, m_fieldRelative);

      // Loop
      if (autoTime >= t)
        m_autoTimer.Reset();

      frc::SmartDashboard::PutNumber("Auto-a", a);
    }
  }

  void TeleopInit() override
  {
    // NOP
  }

  void TeleopPeriodic() override
  {
    // Drivebase
    if (m_controller.IsConnected())
    {
      auto xInput = deadband(m_controller.GetY(frc::GenericHID::kLeftHand), 0.1, 1.0) * -1.0;
      auto yInput = deadband(m_controller.GetX(frc::GenericHID::kLeftHand), 0.1, 1.0) * -1.0;

      auto throttle = m_controller.GetTriggerAxis(frc::GenericHID::kRightHand);

      auto angle = frc::Rotation2d(xInput, yInput);
      if (xInput * xInput + yInput * yInput > 0)
      {
        xInput = angle.Cos() * throttle;
        yInput = angle.Sin() * throttle;
      }

      if (m_controller.GetPOV() != -1)
      {
        auto angle = frc::Rotation2d(units::degree_t{(double)-m_controller.GetPOV()});
        xInput = angle.Cos() * throttle;
        yInput = angle.Sin() * throttle;
      }

      auto rInput = deadband(m_controller.GetX(frc::GenericHID::kRightHand), 0.2, 1.0) * -1.0;
      rInput = rInput * rInput * rInput;

      auto xSpeed = m_xspeedLimiter.Calculate(xInput) * Drivetrain::kMaxSpeed;
      auto ySpeed = m_yspeedLimiter.Calculate(yInput) * Drivetrain::kMaxSpeed;
      auto rot = m_rotLimiter.Calculate(rInput) * Drivetrain::kMaxAngularSpeed;

      m_swerve.Drive(xSpeed, ySpeed, rot, m_fieldRelative);
    }

    // Shooter
    if (m_operator.IsConnected())
    {
      auto gateInput = m_operator.GetTriggerAxis(frc::GenericHID::kRightHand);
      auto gateVoltage = m_gateLimiter.Calculate(gateInput) * Shooter::kMaxGateVoltage;

      m_shooter.SetGate(gateVoltage);

      auto gravityInput = m_operator.GetTriggerAxis(frc::GenericHID::kLeftHand);
      auto gravityVoltage = m_gravityLimiter.Calculate(gravityInput) * Shooter::kMaxGravityVoltage;

      m_shooter.SetGravityBoost(gravityVoltage);

      if (m_operator.GetTouchPadButton())
      {
        m_shooter.GotoSetpoint(0);
      }
      else if (m_operator.GetPOV() == 0)
      {
        m_shooter.GotoSetpoint(1);
      }
      else if (m_operator.GetPOV() == 90)
      {
        m_shooter.GotoSetpoint(2);
      }
      else if (m_operator.GetPOV() == 180)
      {
        m_shooter.GotoSetpoint(3);
      }
      else if (m_operator.GetPOV() == 270)
      {
        m_shooter.GotoSetpoint(4);
      }
    }
  }

  void SimulationPeriodic() override
  {
    m_swerve.SimPeriodic();
  }

private:
  frc::UniversalController m_controller{0};
  frc::UniversalController m_operator{1};

  // Subsystems
  Drivetrain m_swerve;
  Shooter m_shooter;

  // Slew rate limiters to make joystick inputs more gentle
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{10 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{10 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{10 / 1_s};

  frc::SlewRateLimiter<units::scalar> m_shooterLimiter{10 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_gateLimiter{10 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_hoodLimiter{10 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_gravityLimiter{10 / 1_s};

  // Drive Mode
  bool m_fieldRelative = true;

  // Smart Dash
  frc::SendableChooser<std::string> m_chooser;
  static constexpr auto kAutoNone = "0 - None";
  static constexpr auto kAutoBounce = "1 - Bounce";

  static constexpr auto kAutoConstant = "90 - Constant";
  static constexpr auto kAutoToggle = "91 - Toggle";
  static constexpr auto kAutoSweep = "92 - Sweep";
  static constexpr auto kAutoDash = "93 - Dash";

  frc::SendableChooser<frc::UniversalController::ControllerType> m_chooserControllerType;
  frc::SendableChooser<frc::UniversalController::ControllerType> m_chooserOperatorType;
  static constexpr auto kControllerTypePS4 = "PS4";
  static constexpr auto kControllerTypeXbox = "Xbox";
  static constexpr auto kControllerTypeStadia = "Stadia";

  // Auto State
  frc::Timer m_autoTimer;

  // Auto Paths
  frc::Trajectory m_trajectory;

  /// Deadband Function
  double deadband(double input, double band, double max)
  {
    if (fabs(input) < band)
    {
      return 0.0;
    }

    if (input > 0.0)
    {
      return ((input - band) / (1.0 - band)) * max;
    }
    else
    {
      return ((input + band) / (1.0 - band)) * max;
    }
  }

  frc::Trajectory LoadWaypointCSV(std::string path, frc::TrajectoryConfig &config)
  {
    std::vector<frc::Spline<5>::ControlVector> points;

    io::CSVReader<6> csv(path);
    csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
    double x, y, dx, dy, ddx = 0, ddy = 0;
    while (csv.read_row(x, y, dx, dy, ddx, ddy))
    {
      // std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
      points.push_back({{x, dx, ddx}, {y, dy, ddy}});
    }

    return frc::TrajectoryGenerator::GenerateTrajectory(
        points,
        config);
  }
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
