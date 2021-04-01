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

void Drivetrain::Drive(frc::Trajectory::State trajectoryState, units::radian_t yaw)
{
  const auto command = m_trajectoryController.Calculate(
      m_odometry.GetPose(),
      trajectoryState,
      yaw);

  Drive(command.vx, command.vy, command.omega, false);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot,
                       bool fieldRelative)
{
  // Remember the last operating mode, for smartdash display
  m_fieldRelative = fieldRelative;

  // Transform Field Oriented command to a Robot Relative Command
  if (fieldRelative)
  {
    m_command = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetYaw());
  }
  else
  {
    m_command = frc::ChassisSpeeds{xSpeed, ySpeed, rot};
  }

  // Calculate desired swerve states
  auto states = m_kinematics.ToSwerveModuleStates(m_command);
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
  m_odometry.Update(GetYaw(),
                         m_frontLeft.GetState(),
                         m_frontRight.GetState(),
                         m_backLeft.GetState(),
                         m_backRight.GetState());
  
  m_poseEstimator.Update(GetYaw(),
                         m_frontLeft.GetState(),
                         m_frontRight.GetState(),
                         m_backLeft.GetState(),
                         m_backRight.GetState());

  m_robotVelocity = m_kinematics.ToChassisSpeeds({m_frontLeft.GetState(),
                                                  m_frontRight.GetState(),
                                                  m_backLeft.GetState(),
                                                  m_backRight.GetState()});

  m_fieldDisplay.SetRobotPose(m_odometry.GetPose());
}

void Drivetrain::ResetYaw()
{
#ifdef __FRC_ROBORIO__
  m_imu.Reset();
#else
  m_theta = 0_deg;
#endif // __FRC_ROBORIO__
}

frc::Rotation2d Drivetrain::GetYaw()
{
#ifdef __FRC_ROBORIO__
  return frc::Rotation2d{units::degree_t{m_imu.GetAngle()}};
#else
  //return m_gyro.GetRotation2d();
  return m_theta;
#endif
}

void Drivetrain::ResetOdometry(const frc::Pose2d &pose)
{
#ifdef __FRC_ROBORIO__
  m_odometry.ResetPosition(pose, GetYaw());
  m_poseEstimator.ResetPosition(pose, GetYaw());
#else
  m_odometry.ResetPosition(pose, m_theta);
  m_poseEstimator.ResetPosition(pose, m_theta);
#endif
}

void Drivetrain::ShowTrajectory(const frc::Trajectory &trajectory)
{
  vector<frc::Pose2d> poselist;
  for (units::second_t t = 0_s; t <= trajectory.TotalTime(); t += trajectory.TotalTime()/100)
  {
    poselist.push_back(trajectory.Sample(t).pose);
  }
  auto fo = m_fieldDisplay.GetObject("trajectory");
   fo->SetPoses(poselist);
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
  m_theta += m_robotVelocity.omega * 20_ms;
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
      "poseEstimator/x", [this] { return m_poseEstimator.GetEstimatedPosition().X().value(); }, nullptr);
  builder.AddDoubleProperty(
      "poseEstimator/y", [this] { return m_poseEstimator.GetEstimatedPosition().Y().value(); }, nullptr);
  builder.AddDoubleProperty(
      "poseEstimator/yaw", [this] { return m_poseEstimator.GetEstimatedPosition().Rotation().Degrees().value(); }, nullptr);

  builder.AddDoubleProperty(
      "odometry/x", [this] { return m_odometry.GetPose().X().value(); }, nullptr);
  builder.AddDoubleProperty(
      "odometry/y", [this] { return m_odometry.GetPose().Y().value(); }, nullptr);
  builder.AddDoubleProperty(
      "odometry/yaw", [this] { return m_odometry.GetPose().Rotation().Degrees().value(); }, nullptr);

  // Velocity
  builder.AddDoubleProperty(
      "vel/x", [this] { return m_robotVelocity.vx.value(); }, nullptr);
  builder.AddDoubleProperty(
      "vel/y", [this] { return m_robotVelocity.vy.value(); }, nullptr);
  builder.AddDoubleProperty(
      "vel/yaw", [this] { return units::degrees_per_second_t(m_robotVelocity.omega).value(); }, nullptr);

  // Command
  builder.AddDoubleProperty(
      "cmd/x", [this] { return m_command.vx.value(); }, nullptr);
  builder.AddDoubleProperty(
      "cmd/y", [this] { return m_command.vy.value(); }, nullptr);
  builder.AddDoubleProperty(
      "cmd/yaw", [this] { return units::degrees_per_second_t(m_command.omega).value(); }, nullptr);

  // Operating Mode
  builder.AddBooleanProperty(
      "cmd/fieldRelative", [this] { return m_fieldRelative; }, nullptr);
}