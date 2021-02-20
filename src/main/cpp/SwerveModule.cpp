// Copyright (c) FRC3538 - RoboJackets, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/math>

using namespace frc;

SwerveModule::SwerveModule(const int driveMotorChannel,
                           const int turningMotorChannel,
                           const int turningEncoderChannel)
    : m_driveMotor(driveMotorChannel),
      m_turningMotor(turningMotorChannel, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
      m_analogInput(turningEncoderChannel)
{
  // Drive Motor Configuration
  m_driveMotor.ConfigFactoryDefault();
  m_driveMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);
  m_driveMotor.SetInverted(false);
  m_driveMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

  // Turning Motor Configuration
  m_turningMotor.RestoreFactoryDefaults();
  m_turningMotor.SetInverted(false);
  m_turningMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_turningMotor.SetSmartCurrentLimit(kTurningMotorCurrentLimit.value());
  m_turningMotor.EnableVoltageCompensation(kTurningMotorVoltageNominal.value());
  m_turningMotor.BurnFlash();

  // Turning Encoder Config
  m_turningEncoder.SetDistancePerRotation(2 * wpi::math::pi);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(-units::radian_t(wpi::math::pi),
                                               units::radian_t(wpi::math::pi));

  // Restore Offset from preferences
  m_angleOffsetPref += std::to_string(turningEncoderChannel);
  m_angleOffset = frc::Rotation2d(units::degree_t(prefs->GetDouble(m_angleOffsetPref)));
}

frc::SwerveModuleState SwerveModule::GetState()
{
  constexpr int pidIndex = 0;
  auto velocity = m_driveMotor.GetSelectedSensorVelocity(pidIndex) * kDriveScaleFactor / 100_ms;
  auto angle = frc::Rotation2d(units::radian_t(m_turningEncoder.Get()));

  angle += m_angleOffset;

  return {velocity, angle};
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState &state)
{
  // Protect the Neo550 from overheating
  auto temp = units::celsius_t(m_turningMotor.GetMotorTemperature());
  if (m_faultTermal || temp > kTurningMotorTemperatureMax)
  {
    std::cout << "RJ: Neo550 Thermal Fault (Reboot Required) [ " << temp << "C ]" << std::endl;

    m_turningMotor.StopMotor();
    m_driveMotor.StopMotor();

    // Latch this motor off to force someone to figure out what's wrong...
    // Do not use this for competition code; R&D | Home Tasks only
    m_faultTermal = true;
  }

  // Drive Command
  m_drivePIDController.SetPID(
      prefs->GetDouble(kPrefDriveKp, m_drivePIDController.GetP()),
      prefs->GetDouble(kPrefDriveKi, m_drivePIDController.GetI()),
      prefs->GetDouble(kPrefDriveKd, m_drivePIDController.GetD()));

  const auto driveOutput = m_drivePIDController.Calculate(
      GetState().speed,
      state.speed);

  const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

  m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);

  // Turn Command
  m_turningPIDController.SetPID(
      prefs->GetDouble(kPrefTurnKp, m_turningPIDController.GetP()),
      prefs->GetDouble(kPrefTurnKi, m_turningPIDController.GetI()),
      prefs->GetDouble(kPrefTurnKd, m_turningPIDController.GetD()));

  const auto turnOutput = m_turningPIDController.Calculate(
      GetState().angle.Radians(), state.angle.Radians());

  const auto turnFeedforward = m_turnFeedforward.Calculate(
      m_turningPIDController.GetSetpoint().velocity);

  m_turningMotor.SetVoltage(units::volt_t{turnOutput} + turnFeedforward);
}
