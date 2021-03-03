// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

#include <frc/RobotController.h>

Drivetrain::Drivetrain(bool isSimulation)
{
  m_isSimulation = isSimulation;

  // Speed Controllers
  m_driveL0.ConfigFactoryDefault();
  m_driveL1.ConfigFactoryDefault();
  m_driveL2.ConfigFactoryDefault();
  m_driveR0.ConfigFactoryDefault();
  m_driveR1.ConfigFactoryDefault();
  m_driveR2.ConfigFactoryDefault();

  m_rightGroup.SetInverted(true);
  m_leftGroup.SetInverted(false);

  m_driveL0.SetNeutralMode(NeutralMode::Brake);
  m_driveL1.SetNeutralMode(NeutralMode::Brake);
  m_driveL2.SetNeutralMode(NeutralMode::Brake);
  m_driveR0.SetNeutralMode(NeutralMode::Brake);
  m_driveR1.SetNeutralMode(NeutralMode::Brake);
  m_driveR2.SetNeutralMode(NeutralMode::Brake);

  // Encoders
  double pidIdx = 0;
  m_driveL0.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, pidIdx);
  m_driveL0.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);

  m_driveR0.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, pidIdx);
  m_driveR0.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);

  m_driveL0.SetSelectedSensorPosition(0.0);
  m_driveR0.SetSelectedSensorPosition(0.0);

  // IMU
  m_imu.Reset();

  frc::SmartDashboard::PutData("Field", &m_fieldSim);
}

void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds &speeds)
{
  // Feedforward
  auto leftFeedforward = m_feedforward.Calculate(speeds.left);
  auto rightFeedforward = m_feedforward.Calculate(speeds.right);

  // PID
  units::meters_per_second_t leftRate;
  units::meters_per_second_t rightRate;

  if (m_isSimulation)
  {
    leftRate = m_drivetrainSimulator.GetLeftVelocity();
    rightRate = m_drivetrainSimulator.GetLeftVelocity();
  }
  else
  {
    leftRate = m_driveL0.GetSelectedSensorVelocity(0) / 100_ms * kScaleFactor;
    rightRate = m_driveR0.GetSelectedSensorVelocity(0) / 100_ms * kScaleFactor;
  }

  double leftOutput = m_leftPIDController.Calculate(
      leftRate.value(),
      speeds.left.value());

  double rightOutput = m_rightPIDController.Calculate(
      rightRate.value(),
      speeds.right.value());

  // Final Voltage output
  auto left = units::volt_t{leftOutput} + leftFeedforward;
  auto right = units::volt_t{rightOutput} + rightFeedforward;
  // auto left = leftFeedforward;
  // auto right = rightFeedforward;

  m_leftGroup.SetVoltage(left);
  m_rightGroup.SetVoltage(right);

  if (m_isSimulation)
  {
    m_drivetrainSimulator.SetInputs(left, right);
  }
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::radians_per_second_t rot)
{
  SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

void Drivetrain::UpdateOdometry()
{
  if (m_isSimulation)
  {
    m_odometry.Update(m_drivetrainSimulator.GetHeading(),
                      m_drivetrainSimulator.GetLeftPosition(),
                      m_drivetrainSimulator.GetRightPosition());
  } else {
    auto left = m_driveL0.GetSelectedSensorPosition(0) * kScaleFactor;
    auto right = m_driveR0.GetSelectedSensorPosition(0) * kScaleFactor;
    m_odometry.Update(m_imu.GetRotation2d(), left, right);
  }
}

void Drivetrain::ResetOdometry(const frc::Pose2d &pose)
{
  m_driveL0.SetSelectedSensorPosition(0);
  m_driveR0.SetSelectedSensorPosition(0);

  m_drivetrainSimulator.SetPose(pose);
  m_odometry.ResetPosition(pose, pose.Rotation());
}

void Drivetrain::SimulationPeriodic()
{
  m_drivetrainSimulator.Update(20_ms);
}

void Drivetrain::Periodic()
{
  UpdateOdometry();
  m_fieldSim.SetRobotPose(m_odometry.GetPose());

  // double angle = m_imu.GetRotation2d().Degrees();
  // double distL = (m_driveL0.GetSelectedSensorPosition(0) * kScaleFactor);
  // double distR = (m_driveR0.GetSelectedSensorPosition(0) * kScaleFactor);
  // frc::SmartDashboard::PutNumber("Dist L", distL);
  // frc::SmartDashboard::PutNumber("Dist R", distR);
  // frc::SmartDashboard::PutNumber("Angle", angle);
  // frc::SmartDashboard::PutNumber("DISTANCEL", m_driveL0.GetSelectedSensorPosition(0));
  // frc::SmartDashboard::PutNumber("DISTANCER", m_driveR0.GetSelectedSensorPosition(0));
}

void Drivetrain::Log(UDPLogger &logger)
{
  logger.LogExternalDevice(m_driveL0);
  logger.LogExternalDevice(m_driveL1);
  logger.LogExternalDevice(m_driveL2);
  logger.LogExternalDevice(m_driveR0);
  logger.LogExternalDevice(m_driveR1);
  logger.LogExternalDevice(m_driveR2);

#ifdef __FRC_ROBORIO__
  logger.LogExternalDevice(m_imu);
#endif
}