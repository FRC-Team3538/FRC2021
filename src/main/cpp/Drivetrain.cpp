// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

#include <frc/RobotController.h>

void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds) {
  auto leftFeedforward = m_feedforward.Calculate(speeds.left);
  auto rightFeedforward = m_feedforward.Calculate(speeds.right);

  double leftOutput = 0;
  double rightOutput = 0;

  if(m_isSimulation)
  {
    leftOutput = m_leftPIDController.Calculate(
      m_leftEncoder.GetRate(),
      speeds.left.to<double>()
    );

    rightOutput = m_rightPIDController.Calculate(
      m_rightEncoder.GetRate(), 
      speeds.right.to<double>()
      );
  } else {
    leftOutput = m_driveL0.GetSelectedSensorVelocity(0) * m_leftEncoder.GetDistancePerPulse() * 10.0;
    rightOutput = m_driveR0.GetSelectedSensorVelocity(0) * m_leftEncoder.GetDistancePerPulse() * 10.0;
  }

  m_leftGroup.SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
  m_rightGroup.SetVoltage(units::volt_t{rightOutput} + rightFeedforward);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::radians_per_second_t rot) {
  SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

void Drivetrain::UpdateOdometry() {

  if(m_isSimulation)
  {
    m_odometry.Update(m_gyro.GetRotation2d(),
                      units::meter_t(m_leftEncoder.GetDistance()),
                      units::meter_t(m_rightEncoder.GetDistance()));
  } else {
    auto left = m_driveL0.GetSelectedSensorPosition(0) * m_leftEncoder.GetDistancePerPulse();
    auto right = m_driveR0.GetSelectedSensorPosition(0) * m_leftEncoder.GetDistancePerPulse();

    frc::Rotation2d heading;
#ifdef __FRC_ROBORIO__
    heading = frc::Rotation2d(units::degree_t(m_imu.GetAngle()));
#endif
    m_odometry.Update(heading,
                      units::meter_t(left),
                      units::meter_t(right));
  }

}

void Drivetrain::ResetOdometry(const frc::Pose2d& pose) {
  m_leftEncoder.Reset();
  m_rightEncoder.Reset();

  m_driveL0.SetSelectedSensorPosition(0);
  m_driveR0.SetSelectedSensorPosition(0);

  m_drivetrainSimulator.SetPose(pose);
  m_odometry.ResetPosition(pose, pose.Rotation());
}

void Drivetrain::SimulationPeriodic() {
  // To update our simulation, we set motor voltage inputs, update the
  // simulation, and write the simulated positions and velocities to our
  // simulated encoder and gyro. We negate the right side so that positive
  // voltages make the right side move forward.
  m_drivetrainSimulator.SetInputs(units::volt_t{m_leftLeader.Get()} *
                                      frc::RobotController::GetInputVoltage(),
                                  units::volt_t{-m_rightLeader.Get()} *
                                      frc::RobotController::GetInputVoltage());
  m_drivetrainSimulator.Update(20_ms);

  m_leftEncoderSim.SetDistance(
      m_drivetrainSimulator.GetLeftPosition().to<double>());
  m_leftEncoderSim.SetRate(
      m_drivetrainSimulator.GetLeftVelocity().to<double>());
  m_rightEncoderSim.SetDistance(
      m_drivetrainSimulator.GetRightPosition().to<double>());
  m_rightEncoderSim.SetRate(
      m_drivetrainSimulator.GetRightVelocity().to<double>());
  m_gyroSim.SetAngle(
      -m_drivetrainSimulator.GetHeading().Degrees().to<double>());
}

void Drivetrain::Periodic() {
  UpdateOdometry();
  m_fieldSim.SetRobotPose(m_odometry.GetPose());
}
