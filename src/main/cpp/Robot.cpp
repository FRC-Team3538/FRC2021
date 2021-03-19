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
    m_chooser.AddOption(kAutoConstant, kAutoConstant);
    m_chooser.AddOption(kAutoToggle, kAutoToggle);
    m_chooser.AddOption(kAutoSweep, kAutoSweep);
    frc::SmartDashboard::PutData(&m_chooser);

    // Smartdash
    frc::SmartDashboard::PutData("GamepadDriver", &m_controller);
    frc::SmartDashboard::PutData("GamepadOperator", &m_operator);
    // frc::SmartDashboard::PutData("DriveBase", &m_swerve);

    frc::SmartDashboard::SetDefaultNumber("auto/X1", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/Y1", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/R1", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/X2", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/Y2", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/R2", 0.0);
    frc::SmartDashboard::SetDefaultNumber("auto/T", 4.0);

    // UDP Logger
    auto time_point = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(time_point);
    m_udp_logger.SetTitle(std::ctime(&time));
    m_logging_thread =
      std::thread(logToUDPLogger, std::ref(m_udp_logger), std::ref(loggables));
    m_logging_thread.detach();
  }

  void RobotPeriodic() override
  {
       // PS4 | xbox controller mapping
    m_controller.SetControllerType(m_chooserControllerType.GetSelected());
    m_operator.SetControllerType(m_chooserOperatorType.GetSelected());

    // m_swerve.UpdateOdometry();

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

    // // Toggle Drive mode (Field | Robot Relative)
    // if(m_controller.GetShareButtonPressed()) 
    // {
    //   m_swerve.ResetYaw();
    //   std::cout << "Reset Gyro" << std::endl;
    // }
  }

  void AutonomousInit() override
  {
    m_autoTimer.Reset();
    m_autoTimer.Start();
  }

  void AutonomousPeriodic() override
  {
    // // Shared Smartdash Inputs
    // auto x1 = units::meters_per_second_t(frc::SmartDashboard::GetNumber("auto/X1", 0.0));
    // auto y1 = units::meters_per_second_t(frc::SmartDashboard::GetNumber("auto/Y1", 0.0));
    // auto r1 = units::degrees_per_second_t(frc::SmartDashboard::GetNumber("auto/R1", 0.0));

    // auto x2 = units::meters_per_second_t(frc::SmartDashboard::GetNumber("auto/X2", 0.0));
    // auto y2 = units::meters_per_second_t(frc::SmartDashboard::GetNumber("auto/Y2", 0.0));
    // auto r2 = units::degrees_per_second_t(frc::SmartDashboard::GetNumber("auto/R2", 0.0));

    // auto t = units::second_t(frc::SmartDashboard::GetNumber("auto/T", 4.0));
    // auto autoTime = units::second_t(m_autoTimer.Get());

    // //
    // // Program Selection
    // //
    // auto program = m_chooser.GetSelected();
    // if (program == kAutoNone)
    // {
    //   return;
    // }
    // else if (program == kAutoConstant)
    // {
    //   // Just go at a fixed command
    //   m_swerve.Drive(x1, y1, r1, m_fieldRelative);
    // }
    // else if (program == kAutoToggle)
    // {
    //   // Toggle between two setpoints
    //   if(autoTime > t/2)
    //   {
    //     m_swerve.Drive(x1, y1, r1, m_fieldRelative);
    //   } else {
    //     m_swerve.Drive(x2, y2, r2, m_fieldRelative);
    //   }

    //   // Loop
    //   if(autoTime >= t) m_autoTimer.Reset();
    // }
    // else if (program == kAutoSweep)
    // {
    //   // Protect div0
    //   if(t <= 0.0_s) return;

    //   auto a = units::math::abs(autoTime/t - 0.5) * 2.0;
    //   auto x = a * x1 + (1.0 - a) * x2;
    //   auto y = a * y1 + (1.0 - a) * y2;
    //   auto r = a * r1 + (1.0 - a) * r2;

    //   m_swerve.Drive(x, y, r, m_fieldRelative);

    //   // Loop
    //   if(autoTime >= t) m_autoTimer.Reset();

    //   frc::SmartDashboard::PutNumber("Auto-a", a);
    // }
  }

  void TeleopInit() override
  {
    // std::cout << "time,xSpeed,ySpeed,rot,FL_CurrentVelocity,FL_GoalVelocity,FL_drivePIDOutput,FL_drivePIDVelocity,FL_drivePIDAccel,FL_driveFF,FL_CurrentAngle,FL_GoalAngle,FL_turnPIDOutput,FL_turningSetpointPosition,FL_turningSetpointVelocity,FL_turnFeedforward,FR_CurrentVelocity,FR_GoalVelocity,FR_drivePIDOutput,FR_drivePIDVelocity,FR_drivePIDAccel,FR_driveFF,FR_CurrentAngle,FR_GoalAngle,FR_turnPIDOutput,FR_turningSetpointPosition,FR_turningSetpointVelocity,FR_turnFeedforward,BL_CurrentVelocity,BL_GoalVelocity,BL_drivePIDOutput,BL_drivePIDVelocity,BL_drivePIDAccel,BL_driveFF,BL_CurrentAngle,BL_GoalAngle,BL_turnPIDOutput,BL_turningSetpointPosition,BL_turningSetpointVelocity,BL_turnFeedforward,BR_CurrentVelocity,BR_GoalVelocity,BR_drivePIDOutput,BR_drivePIDVelocity,BR_drivePIDAccel,BR_driveFF,BR_CurrentAngle,BR_GoalAngle,BR_turnPIDOutput,BR_turningSetpointPosition,BR_turningSetpointVelocity,BR_turnFeedforward," << std::endl;
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

    // // Drivebase
    // auto xInput = deadband(m_controller.GetY(frc::GenericHID::kLeftHand), 0.1, 1.0) * -1.0;
    // auto yInput = deadband(m_controller.GetX(frc::GenericHID::kLeftHand), 0.1, 1.0) * 1.0;
    // auto rInput = deadband(m_controller.GetX(frc::GenericHID::kRightHand), 0.1, 1.0) * 1.0;

    // if (xInput * xInput + yInput * yInput > 0) {
    //   auto throttle = m_controller.GetTriggerAxis(frc::GenericHID::kRightHand);
    //   // throttle = std::sqrt(xInput*xInput + yInput*yInput);
    //   auto angle = frc::Rotation2d(xInput, yInput);
    //   xInput = angle.Cos() * throttle;
    //   yInput = angle.Sin() * throttle;
    // }

    // if (rInput < 0) {
    //   rInput = -rInput * rInput;
    // } else {
    //   rInput = rInput * rInput;
    // }

    // auto xSpeed = m_xspeedLimiter.Calculate(xInput) * Drivetrain::kMaxSpeed;
    // auto ySpeed = m_yspeedLimiter.Calculate(yInput) * Drivetrain::kMaxSpeed;
    // auto rot = m_rotLimiter.Calculate(rInput) * Drivetrain::kMaxAngularSpeed;
    
    // m_swerve.Drive(xSpeed, ySpeed, rot, m_fieldRelative);
  }

  void SimulationPeriodic() override
  {
    // m_swerve.SimPeriodic();
  }

private:
  frc::UniversalController m_controller{0};
  frc::UniversalController m_operator{1};

  // Drivetrain m_swerve;
  GlobalDevices m_globals;
  Shooter m_shooter;

  UDPLogger m_udp_logger;
  std::thread m_logging_thread;

  std::vector<std::shared_ptr<rj::Loggable>> loggables{
    std::shared_ptr<rj::Loggable>(&m_globals),
    // std::shared_ptr<rj::Loggable>(&m_swerve),
    std::shared_ptr<rj::Loggable>(&m_shooter),
  };



  // SwerveModule module{1, 2, 3};

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
  static constexpr auto kAutoConstant = "1 - Constant";
  static constexpr auto kAutoToggle = "2 - Toggle";
  static constexpr auto kAutoSweep = "3 - Sweep";

  frc::SendableChooser<frc::UniversalController::ControllerType> m_chooserControllerType;
  frc::SendableChooser<frc::UniversalController::ControllerType> m_chooserOperatorType;
  static constexpr auto kControllerTypePS4 = "PS4";
  static constexpr auto kControllerTypeXbox = "Xbox";
  static constexpr auto kControllerTypeStadia = "Stadia";

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
