// Copyright (c) FRC3538 - RoboJackets, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain()
{
#ifdef __FRC_ROBORIO__
  m_imu.Reset();
#else
  m_gyro.Reset();
#endif

  frc::SmartDashboard::PutData("Field", &m_fieldDisplay);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot,
                       bool fieldRelative)
{
  // Field / Robot Relative Control
  frc::ChassisSpeeds cmd;
  if (fieldRelative)
  {
    cmd = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetYaw());
  }
  else
  {
    cmd = frc::ChassisSpeeds{xSpeed, ySpeed, rot};
  }

  // Calculate desired swerve states
  auto states = m_kinematics.ToSwerveModuleStates(cmd);
  m_kinematics.NormalizeWheelSpeeds(&states, kMaxSpeed);

  std::cout << frc::Timer::GetFPGATimestamp() << "," << xSpeed.value() << "," << ySpeed.value() << "," << rot.value() << ",";

  // Set State of Each Module
  auto [fl, fr, bl, br] = states;
  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
  std::cout << std::endl;
}

void Drivetrain::UpdateOdometry()
{
  m_poseEstimator.Update(GetYaw(),
                         m_frontLeft.GetState(),
                         m_frontRight.GetState(),
                         m_backLeft.GetState(),
                         m_backRight.GetState());

  m_fieldDisplay.SetRobotPose(m_poseEstimator.GetPose());
}

void Drivetrain::ResetYaw()
{
#ifdef __FRC_ROBORIO__
    m_imu.Reset();
#endif // __FRC_ROBORIO__
}

frc::Rotation2d Drivetrain::GetYaw()
{
#ifdef __FRC_ROBORIO__
  return m_imu.GetRotation2d();
#else
  //return m_gyro.GetRotation2d();
  return m_theta;
#endif
}

void Drivetrain::Log(UDPLogger &logger)
{
  m_frontLeft.Log(logger);
  m_frontRight.Log(logger);
  m_backLeft.Log(logger);
  m_backRight.Log(logger);
#ifdef __FRC_ROBORIO__
  logger.LogExternalDevice(m_imu);
#endif // __FRC_ROBORIO__
}


void Drivetrain::SimPeriodic()
{
  m_frontLeft.SimPeriodic();
  m_frontRight.SimPeriodic();
  m_backLeft.SimPeriodic();
  m_backRight.SimPeriodic();

  // Simulated IMU
  auto [dx, dy, dtheta] = m_kinematics.ToChassisSpeeds({
    m_frontLeft.GetState(),
    m_frontRight.GetState(),
    m_backLeft.GetState(),
    m_backRight.GetState()});
  
  m_theta += dtheta * 20_ms;
  (void)dx;
  (void)dy;
}

void Drivetrain::InitSendable(frc::SendableBuilder &builder)
{
  builder.SetSmartDashboardType("DriveBase");
  builder.SetActuator(true);

  // Modules
  m_frontLeft.InitSendable(builder, "FL");
  m_frontRight.InitSendable(builder, "FR");
  m_backLeft.InitSendable(builder, "BL");
  m_backRight.InitSendable(builder, "BR");
  
  // Pose
  builder.AddDoubleProperty(
      "pose/x", [this] { return m_poseEstimator.GetPose().X().value(); }, nullptr);
  builder.AddDoubleProperty(
      "pose/y", [this] { return m_poseEstimator.GetPose().Y().value(); }, nullptr);
  builder.AddDoubleProperty(
      "pose/yaw", [this] { return m_poseEstimator.GetPose().Rotation().Degrees().value(); }, nullptr);
}