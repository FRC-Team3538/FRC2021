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

  frc::SmartDashboard::PutData("SwerveFL", &m_frontLeft);
  frc::SmartDashboard::PutData("SwerveFR", &m_frontRight);
  frc::SmartDashboard::PutData("SwerveBL", &m_backLeft);
  frc::SmartDashboard::PutData("SwerveBR", &m_backRight);
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

  // Set State of Each Module
  auto [fl, fr, bl, br] = states;
  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void Drivetrain::UpdateOdometry()
{
  m_poseEstimator.Update(GetYaw(),
                         m_frontLeft.GetState(),
                         m_frontRight.GetState(),
                         m_backLeft.GetState(),
                         m_backRight.GetState());
}

frc::Rotation2d Drivetrain::GetYaw()
{

#ifdef __FRC_ROBORIO__
  return m_imu.GetRotation2d();
#else
  return m_gyro.GetRotation2d();
#endif
}

void Drivetrain::Log()
{
}
