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

#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryUtil.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/GlobalDevices.h"
#include "subsystems/SwerveModule.h"
#include "subsystems/Shooter.h"
#include "lib/UniversalController.hpp"
#include <UDPLogger.hpp>
#include <cmath>

#include <memory>
#include <thread>

void
logToUDPLogger(UDPLogger& logger, std::vector<std::shared_ptr<rj::Loggable>>& loggables)
{
  auto target =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
  logger.InitLogger();
  while (true) {
    logger.CheckForNewClient();

    for (auto& loggable : loggables) {
      loggable->Log(logger);
    }
    logger.FlushLogBuffer();

    std::this_thread::sleep_until(target);
    target = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
  }
}

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
    m_chooser.AddOption(kAutoARed, kAutoARed);
    m_chooser.AddOption(kAutoABlue, kAutoABlue);
    m_chooser.AddOption(kAutoBRed, kAutoBRed);
    m_chooser.AddOption(kAutoBBlue, kAutoBBlue);
    m_chooser.AddOption(kAutoBarrel, kAutoBarrel);
    m_chooser.AddOption(kAutoSlalom, kAutoSlalom);
    m_chooser.AddOption(kAutoAccuracy, kAutoAccuracy);
    m_chooser.AddOption(kAutoPowerPort, kAutoPowerPort);

    m_chooser.AddOption(kAutoConstant, kAutoConstant);
    m_chooser.AddOption(kAutoToggle, kAutoToggle);
    m_chooser.AddOption(kAutoSweep, kAutoSweep);
    frc::SmartDashboard::PutData(&m_chooser);

    // Smartdash
    frc::SmartDashboard::PutData("GamepadDriver", &m_controller);
    frc::SmartDashboard::PutData("GamepadOperator", &m_operator);
    frc::SmartDashboard::PutData("DriveBase", &m_swerve);

    frc::SmartDashboard::SetDefaultNumber("auto/A_X", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/A_Y", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/A_R", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/B_X", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/B_Y", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/B_R", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/T", 4.0);

    // UDP Logger
    auto time_point = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(time_point);
    m_udp_logger.SetTitle(std::ctime(&time));
    m_logging_thread =
      std::thread(logToUDPLogger, std::ref(m_udp_logger), std::ref(loggables));
    m_logging_thread.detach();
  }

  void DisabledInit() override
  {
    // Mostly for sim
    m_swerve.Drive(0_mps, 0_mps, 0_deg_per_s, false);
  }

  void RobotPeriodic() override
  {
       // PS4 | xbox controller mapping
    m_controller.SetControllerType(m_chooserControllerType.GetSelected());
    m_operator.SetControllerType(m_chooserOperatorType.GetSelected());

    // Update Drivebase Odometry
    m_swerve.UpdateOdometry();

    // Toggle Drive mode (Field | Robot Relative)
    if(m_controller.GetOptionsButtonPressed()) 
    {
      m_fieldRelative = !m_fieldRelative;
      if (m_fieldRelative) {
        std::cout << "Switching to Field Relative Control" << std::endl;
      }
      else
      {
        std::cout << "Switching to Normal Control" << std::endl;
      }
    }

    // Toggle Drive mode (Field | Robot Relative)
    if(m_controller.GetShareButtonPressed()) 
    {
      m_swerve.ResetYaw();
      std::cout << "Reset Gyro" << std::endl;
    }
  }

  void AutonomousInit() override
  {
    m_autoTimer.Reset();
    m_autoTimer.Start();

    //
    // Program Selection
    //
    auto program = m_chooser.GetSelected();
    if (program == kAutoARed)
    {
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d(30_in, 90_in, 0_deg), 
          {
            // frc::Translation2d(150_in, 60_in), 
            // frc::Translation2d(180_in, 150_in)
          },
          frc::Pose2d(330_in, 60_in, 0_deg),
          frc::TrajectoryConfig(3_fps, 3_fps_sq));

      // Set initial pose of robot
      m_swerve.ResetYaw();
      m_swerve.ResetOdometry(m_trajectory.InitialPose());
    }
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
    else if (program == kAutoARed)
    {
      m_swerve.Drive(m_trajectory, units::second_t(m_autoTimer.Get()));
    }
    else if (program == kAutoConstant)
    {
      // Just go at a fixed command
      m_swerve.Drive(x1, y1, r1, m_fieldRelative);
    }
    else if (program == kAutoToggle)
    {
      // Toggle between two setpoints
      if(autoTime > t/2)
      {
        m_swerve.Drive(x1, y1, r1, m_fieldRelative);
      } else {
        m_swerve.Drive(x2, y2, r2, m_fieldRelative);
      }

      // Loop
      if(autoTime >= t) m_autoTimer.Reset();
    }
    else if (program == kAutoSweep)
    {
      // Protect div0
      if(t <= 0.0_s) return;

      auto a = units::math::abs(autoTime/t - 0.5) * 2.0;
      auto x = a * x1 + (1.0 - a) * x2;
      auto y = a * y1 + (1.0 - a) * y2;
      auto r = a * r1 + (1.0 - a) * r2;

      m_swerve.Drive(x, y, r, m_fieldRelative);

      // Loop
      if(autoTime >= t) m_autoTimer.Reset();

      frc::SmartDashboard::PutNumber("Auto-a", a);
    }
  }

  void TeleopInit() override
  {
    // NOP
  }

  void TeleopPeriodic() override
  {
    // Shooter
    auto shooterInput = m_operator.GetTriggerAxis(frc::GenericHID::kLeftHand);
    auto gateInput = m_operator.GetTriggerAxis(frc::GenericHID::kRightHand);
    auto hoodInput = deadband(m_operator.GetX(frc::GenericHID::kLeftHand), 0.1, 1.0);

    auto shooterVoltage = m_shooterLimiter.Calculate(shooterInput) * Shooter::kMaxShooterVoltage;
    auto gateVoltage = m_gateLimiter.Calculate(gateInput) * Shooter::kMaxGateVoltage;
    auto hoodVoltage = m_hoodLimiter.Calculate(hoodInput) * Shooter::kMaxHoodVoltage;

    m_shooter.Set(shooterVoltage, gateVoltage, hoodVoltage);

    // Drivebase
    auto xInput = deadband(m_controller.GetY(frc::GenericHID::kLeftHand), 0.1, 1.0) * -1.0;
    auto yInput = deadband(m_controller.GetX(frc::GenericHID::kLeftHand), 0.1, 1.0) * -1.0;
    auto rInput = deadband(m_controller.GetX(frc::GenericHID::kRightHand), 0.1, 1.0) * -1.0;

    if (xInput * xInput + yInput * yInput > 0) {
      auto throttle = m_controller.GetTriggerAxis(frc::GenericHID::kRightHand);
      // throttle = std::sqrt(xInput*xInput + yInput*yInput);
      auto angle = frc::Rotation2d(xInput, yInput);
      xInput = angle.Cos() * throttle;
      yInput = angle.Sin() * throttle;
    }

    // Increase Low-end stick control ("Smoothing")
    if (rInput < 0) {
      rInput = -rInput * rInput;
    } else {
      rInput = rInput * rInput;
    }

    auto xSpeed = m_xspeedLimiter.Calculate(xInput) * Drivetrain::kMaxSpeed;
    auto ySpeed = m_yspeedLimiter.Calculate(yInput) * Drivetrain::kMaxSpeed;
    auto rot = m_rotLimiter.Calculate(rInput) * 360_deg_per_s;
    
    m_swerve.Drive(xSpeed, ySpeed, rot, m_fieldRelative);
  }

  void SimulationPeriodic() override
  {
    m_swerve.SimPeriodic();
  }

private:
  frc::UniversalController m_controller{0};
  frc::UniversalController m_operator{1};

  Drivetrain m_swerve;
  GlobalDevices m_globals;
  Shooter m_shooter;

  UDPLogger m_udp_logger;
  std::thread m_logging_thread;

  std::vector<std::shared_ptr<rj::Loggable>> loggables{
    std::shared_ptr<rj::Loggable>(&m_globals),
    // std::shared_ptr<rj::Loggable>(&m_swerve),
    std::shared_ptr<rj::Loggable>(&m_shooter),
  };

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{10 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{10 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{10 / 1_s};

  frc::SlewRateLimiter<units::scalar> m_shooterLimiter{10 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_gateLimiter{10 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_hoodLimiter{10 / 1_s};

  // Drive Mode
  bool m_fieldRelative = true;

  // Smart Dash
  frc::SendableChooser<std::string> m_chooser;
  static constexpr auto kAutoNone = "0 - None";
  static constexpr auto kAutoARed = "1 - A Red";
  static constexpr auto kAutoABlue = "2 - A Blue";
  static constexpr auto kAutoBRed = "3 - B Red";
  static constexpr auto kAutoBBlue = "4 - B Blue";
  static constexpr auto kAutoBarrel = "5 - Barrel";
  static constexpr auto kAutoSlalom = "6 - Slalom";
  static constexpr auto kAutoAccuracy = "7 - Accuracy";
  static constexpr auto kAutoPowerPort = "8 - PowerPort";

  static constexpr auto kAutoConstant = "90 - Constant";
  static constexpr auto kAutoToggle = "91 - Toggle";
  static constexpr auto kAutoSweep = "92 - Sweep";

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

    if(input > 0.0)
    {
      return ((input - band) / (1.0 - band)) * max;
    } else {
      return ((input + band) / (1.0 - band)) * max;
    }
  }
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
