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

#include "subsystems/Drivetrain.h"
#include "subsystems/GlobalDevices.h"
#include "subsystems/SwerveModule.h"
#include "lib/UniversalController.hpp"
#include <UDPLogger.hpp>

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

    auto time = frc::Timer::GetFPGATimestamp();
    for (auto& loggable : loggables) {
      loggable->Log(logger);
    }
    logger.FlushLogBuffer();

    // std::cout << "finished log loop" << std::endl;
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

    // PS4 | xbox controller mapping
    m_controller.SetControllerType(frc::UniversalController::ControllerType::kPS4);

    // Auto Program Selection
    m_chooser.SetDefaultOption(kAutoNone, kAutoNone);
    m_chooser.AddOption(kAutoConstant, kAutoConstant);
    m_chooser.AddOption(kAutoToggle, kAutoToggle);
    m_chooser.AddOption(kAutoSweep, kAutoSweep);
    frc::SmartDashboard::PutData(&m_chooser);

    // Smartdash
    frc::SmartDashboard::SetDefaultNumber("auto/X1", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/Y1", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/R1", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/X2", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/Y2", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/R2", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/T", 4.0);

    auto time_point = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(time_point);
    m_udp_logger.SetTitle(std::ctime(&time));
    m_logging_thread =
      std::thread(logToUDPLogger, std::ref(m_udp_logger), std::ref(loggables));
    m_logging_thread.detach();
  }

  void RobotPeriodic() override
  {
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
  }

  void AutonomousInit() override
  {
    m_autoTimer.Reset();
    m_autoTimer.Start();
  }

  void AutonomousPeriodic() override
  {
    // Shared Smartdash Inputs
    auto x1 = units::meters_per_second_t(frc::SmartDashboard::GetNumber("auto/X1", 0.0));
    auto y1 = units::meters_per_second_t(frc::SmartDashboard::GetNumber("auto/Y1", 0.0));
    auto r1 = units::degrees_per_second_t(frc::SmartDashboard::GetNumber("auto/R1", 0.0));

    auto x2 = units::meters_per_second_t(frc::SmartDashboard::GetNumber("auto/X2", 0.0));
    auto y2 = units::meters_per_second_t(frc::SmartDashboard::GetNumber("auto/Y2", 0.0));
    auto r2 = units::degrees_per_second_t(frc::SmartDashboard::GetNumber("auto/R2", 0.0));

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

  void TeleopPeriodic() override
  {
    // Drivebase
    // NOTE: controller axes / robot axes are flipped 
    // because the robot considers x downfield, which is y to the controller
    // TODO: Calibrate this for ps4 controllers
    const auto ySpeed = m_xspeedLimiter.Calculate(deadband(m_controller.GetX(frc::GenericHID::kLeftHand), 0.1, 1)) * Drivetrain::kMaxSpeed;
    const auto xSpeed = -m_yspeedLimiter.Calculate(deadband(m_controller.GetY(frc::GenericHID::kLeftHand), 0.1, 1)) * Drivetrain::kMaxSpeed;
    const auto rot = m_rotLimiter.Calculate(deadband(m_controller.GetX(frc::GenericHID::kRightHand), 0.1, 1)) * Drivetrain::kMaxAngularSpeed;
    m_swerve.Drive(xSpeed, ySpeed, rot, m_fieldRelative);

    // const auto drive = m_controller.GetY(frc::GenericHID::kLeftHand) * Drivetrain::kMaxSpeed;
    // const auto rot_x = units::meter_t(m_controller.GetX(frc::GenericHID::kRightHand));
    // const auto rot_y = units::meter_t(m_controller.GetY(frc::GenericHID::kRightHand));
    // const auto ang = units::math::atan2(rot_y, rot_x);
    // frc::SwerveModuleState state{drive, frc::Rotation2d(ang)};
    // module.SetDesiredState(state);
  }

private:
  frc::UniversalController m_controller{0};
  Drivetrain m_swerve;
  GlobalDevices m_globals;

  UDPLogger m_udp_logger;
  std::thread m_logging_thread;

  std::vector<std::shared_ptr<rj::Loggable>> loggables{
    std::shared_ptr<rj::Loggable>(&m_globals),
    std::shared_ptr<rj::Loggable>(&m_swerve),
  };



  // SwerveModule module{1, 2, 3};

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{10 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{10 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{10 / 1_s};

  // Drive Mode
  bool m_fieldRelative = true;

  // Smart Dash
  frc::SendableChooser<std::string> m_chooser;
  static constexpr auto kAutoNone = "0 - None";
  static constexpr auto kAutoConstant = "1 - Constant";
  static constexpr auto kAutoToggle = "2 - Toggle";
  static constexpr auto kAutoSweep = "3 - Sweep";

  // Auto State
  frc::Timer m_autoTimer;

  double deadband(double input, double band, double max)
  {
    if (input < band && -input < band) 
    {
      return 0;
    }
    return (input - band) / (max - band);
  }
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
