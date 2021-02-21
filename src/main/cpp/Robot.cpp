// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/SlewRateLimiter.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "Drivetrain.h"
#include "UniversalController.hpp"

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override
  {
    // Flush NetworkTables every loop.
    // This is probably not desireable for comp code, but good for debugging
    SetNetworkTablesFlushEnabled(true);

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
  }

  void RobotPeriodic() override
  {
    m_swerve.UpdateOdometry();
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
    const auto xSpeed = -m_xspeedLimiter.Calculate(m_controller.GetX(frc::GenericHID::kLeftHand)) * Drivetrain::kMaxSpeed;
    const auto ySpeed = -m_yspeedLimiter.Calculate(m_controller.GetY(frc::GenericHID::kLeftHand)) * Drivetrain::kMaxSpeed;
    const auto rot = -m_rotLimiter.Calculate(m_controller.GetX(frc::GenericHID::kRightHand)) * Drivetrain::kMaxAngularSpeed;
    m_swerve.Drive(xSpeed, ySpeed, rot, m_fieldRelative);

    // Toggle Drive mode (Field | Robot Relative)
    if(m_controller.GetOptionsButtonPressed()) m_fieldRelative = !m_fieldRelative;
  }

private:
  frc::UniversalController m_controller{0};
  Drivetrain m_swerve;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

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
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
