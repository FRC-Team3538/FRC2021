/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Drivetrain.h"

#include <frc/RobotController.h>

void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds &speeds)
{
  auto leftFeedforward = m_feedforward.Calculate(speeds.left);
  auto rightFeedforward = m_feedforward.Calculate(speeds.right);
  double leftOutput = m_leftPIDController.Calculate(GetEncoderVelocityLeft().value(),
                                                    speeds.left.to<double>());
  double rightOutput = m_rightPIDController.Calculate(
      GetEncoderVelocityRight().value(), speeds.right.to<double>());

  m_leftGroup.SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
  m_rightGroup.SetVoltage(units::volt_t{rightOutput} + rightFeedforward);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::radians_per_second_t rot)
{
  SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

void Drivetrain::UpdateOdometry()
{
  m_odometry.Update(
#ifdef __FRC_ROBORIO__
      m_imu.GetRotation2d(),
#else
      m_gyro.GetRotation2d(),
#endif
      GetEncoderPositionLeft(),
      GetEncoderPositionRight());
}

void Drivetrain::ResetOdometry(const frc::Pose2d &pose)
{
  SetEncoderPositionLeft(0_m);
  SetEncoderPositionRight(0_m);
  m_drivetrainSimulator.SetPose(pose);
  m_odometry.ResetPosition(pose, pose.Rotation());
}

void Drivetrain::SimulationPeriodic()
{
  // To update our simulation, we set motor voltage inputs, update the
  // simulation, and write the simulated positions and velocities to our
  // simulated encoder and gyro. We negate the right side so that positive
  // voltages make the right side move forward.
  m_drivetrainSimulator.SetInputs(units::volt_t{m_leftLeader.Get()} *
                                      frc::RobotController::GetInputVoltage(),
                                  units::volt_t{-m_rightLeader.Get()} *
                                      frc::RobotController::GetInputVoltage());
  m_drivetrainSimulator.Update(20_ms);
  SetEncoderPositionLeft(m_drivetrainSimulator.GetLeftPosition());
  //m_leftEncoderSim.SetRate(
  //m_drivetrainSimulator.GetLeftVelocity().to<double>());
  SetEncoderPositionRight(m_drivetrainSimulator.GetRightPosition());
  //m_rightEncoderSim.SetRate(
  //m_drivetrainSimulator.GetRightVelocity().to<double>());
  m_gyroSim.SetAngle(
      -m_drivetrainSimulator.GetHeading().Degrees().to<double>());
}

units::meter_t Drivetrain::GetEncoderPositionLeft()
{
#ifdef __FRC_ROBORIO__
  return m_DriveL0.GetSelectedSensorPosition(0) * kScaleFactor;
#else
  return m_leftLeader.GetSelectedSensorPosition(0) * kScaleFactor;
#endif
}

units::meter_t Drivetrain::GetEncoderPositionRight()
{
#ifdef __FRC_ROBORIO__
  return m_DriveR0.GetSelectedSensorPosition(0) * kScaleFactor;
#else
  return m_rightLeader.GetSelectedSensorPosition(0) * kScaleFactor;
#endif
}
units::meters_per_second_t Drivetrain::GetEncoderVelocityLeft()
{
#ifdef __FRC_ROBORIO__
  return m_DriveL0.GetSelectedSensorVelocity(0) * kScaleFactor / 0.1_s;
#else
  return m_drivetrainSimulator.GetLeftVelocity();
#endif
}
units::meters_per_second_t Drivetrain::GetEncoderVelocityRight()
{
#ifdef __FRC_ROBORIO__
  return m_DriveR0.GetSelectedSensorVelocity(0) * kScaleFactor / 0.1_s;
#else
  return m_drivetrainSimulator.GetRightVelocity();
#endif
}
void Drivetrain::SetEncoderPositionLeft(units::meter_t position)
{
  m_leftLeader.SetSelectedSensorPosition((position / kScaleFactor).value());
  m_DriveL0.SetSelectedSensorPosition((position / kScaleFactor).value());
}
void Drivetrain::SetEncoderPositionRight(units::meter_t position)
{
  m_rightLeader.SetSelectedSensorPosition((position / kScaleFactor).value());
  m_DriveR0.SetSelectedSensorPosition((position / kScaleFactor).value());
}

void Drivetrain::Periodic()
{
  UpdateOdometry();
  m_fieldSim.SetRobotPose(m_odometry.GetPose());
}
