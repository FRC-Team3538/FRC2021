// Copyright (c) FRC3538 - RoboJackets, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/math>
#include <iostream>
#include <cmath>
#include <frc/MathUtil.h>

using namespace frc;

SwerveModule::SwerveModule(const int driveMotorChannel,
                           const int turningMotorChannel,
                           const int turningEncoderChannel,
                           SwerveModuleConfig config)
    : m_driveMotor(driveMotorChannel),
      m_turningMotor(turningMotorChannel),
      m_turningEncoder(turningEncoderChannel),
      m_drivePIDController{config.drivePID.kP, config.drivePID.kI, config.drivePID.kD, {config.drivePID.max_acceleration, config.drivePID.max_jerk}},
      m_turningPIDController{config.turningPID.kP, config.turningPID.kI, config.turningPID.kD, {config.turningPID.max_angular_velocity, config.turningPID.max_angular_acceleration}},
      m_driveFeedforward{config.driveFf.kS, config.driveFf.kV, config.driveFf.kA},
      m_turnFeedforward{config.turnFf.kS, config.turnFf.kV, config.turnFf.kA}
{
  m_turningPIDController.SetTolerance(0.1_rad, units::radians_per_second_t(std::numeric_limits<double>::infinity()));
  // Drive Motor Configuration
  m_driveMotor.ConfigFactoryDefault();
  m_driveMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);
  m_driveMotor.SetInverted(false); // Remember: forward-positive!
  m_driveMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_driveMotor.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 55, 55, 0.0));

  // Turning Motor Configuration
  m_turningMotor.ConfigFactoryDefault();
  m_turningMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);
  m_turningMotor.SetInverted(false); // Remember: forward-positive!
  m_turningMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_turningMotor.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 30, 30, 0.0));

  // Turning Encoder Config
  ctre::phoenix::sensors::CANCoderConfiguration encoderConfig;
  m_turningEncoder.GetAllConfigs(encoderConfig);
  encoderConfig.enableOptimizations = true;
  encoderConfig.initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
  encoderConfig.absoluteSensorRange = ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180;
  encoderConfig.magnetOffsetDegrees = config.angleOffset.value();
  m_turningEncoder.ConfigAllSettings(encoderConfig);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(-units::radian_t(wpi::math::pi),
                                               units::radian_t(wpi::math::pi));
  m_turningPIDController.Reset(units::degree_t(m_turningEncoder.GetAbsolutePosition()));

  // Use mag encoder to set start-up angle
  // m_neoEncoder.SetPosition(0);
  // m_turningEncoder.GetPosition() / 360);
}

frc::SwerveModuleState SwerveModule::GetState()
{
  return {GetVelocity(), GetAngle()};
}

units::meters_per_second_t SwerveModule::GetVelocity()
{
  if (m_isSimulation)
  {
    // Simulator
    return m_driveSim.GetVelocity();
  }
  else
  {
    // Real Hardware
    constexpr int pidIndex = 0;
    return m_driveMotor.GetSelectedSensorVelocity(pidIndex) * kDriveScaleFactor / 100_ms;
  }
}

frc::Rotation2d SwerveModule::GetAngle()
{
  if (m_isSimulation)
  {
    // Simulator
    auto un_normalized = frc::Rotation2d(units::degree_t(m_turnSim.GetPosition()));
    return frc::Rotation2d(un_normalized.Cos(), un_normalized.Sin());
  }
  else
  {
    // Real Hardware
    auto un_normalized = frc::Rotation2d(units::degree_t(m_turningEncoder.GetAbsolutePosition()));
    return frc::Rotation2d(un_normalized.Cos(), un_normalized.Sin());
  }
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState &state)
{
  const auto currentState = GetState();

  // Protect the Neo550 from overheating
  auto temp = units::celsius_t(m_turningMotor.GetTemperature());
  if (m_faultTermal || temp > kTurningMotorTemperatureMax)
  {
    std::cout << "RJ: Neo550 Thermal Fault (Reboot Required) [ " << temp << "C ]" << std::endl;

    m_turningMotor.StopMotor();
    m_driveMotor.StopMotor();

    // Latch this motor off to force someone to figure out what's wrong...
    // Do not use this for competition code; R&D | Home Tasks only
    m_faultTermal = true;
    return;
  }

  // Forward Kinematics
  const auto opt_state = SwerveModuleState::Optimize(state, currentState.angle);

  // Drive
  const auto driveOutput = m_drivePIDController.Calculate(
      currentState.speed,
      opt_state.speed);

  const auto driveFeedforward = m_driveFeedforward.Calculate(m_drivePIDController.GetSetpoint().position, m_drivePIDController.GetSetpoint().velocity);

  m_driveVolts = units::volt_t{driveOutput} + driveFeedforward;

  // Angle
  const auto turnOutput = m_turningPIDController.Calculate(
      currentState.angle.Radians(),
      opt_state.angle.Radians());

  const auto turnFeedforward = m_turnFeedforward.Calculate(
      m_turningPIDController.GetSetpoint().velocity);

  m_turnVolts = units::volt_t{turnOutput} + turnFeedforward;

  // if we're moving at less than an inch per second just ignore
  if (units::math::abs(state.speed) < units::feet_per_second_t(1.0 / 12.0))
  {
    m_turnVolts = 0_V;
    m_driveVolts = 0_V;
    m_turningPIDController.Reset(currentState.angle.Radians());
    m_drivePIDController.Reset(currentState.speed);
  }

  // Output
  m_driveMotor.SetVoltage(m_driveVolts);
  m_turningMotor.SetVoltage(m_turnVolts);

  if (m_isSimulation)
  {
    m_driveSim.SetInputVoltage(m_driveVolts);
    m_turnSim.SetInputVoltage(m_turnVolts);
  }
}

void SwerveModule::SimPeriodic()
{
  m_isSimulation = true;

  m_driveSim.Update(20_ms);
  m_turnSim.Update(20_ms);
}

void SwerveModule::Log(UDPLogger &logger)
{
  logger.LogExternalDevice(m_driveMotor);
  logger.LogExternalDevice(m_turningMotor);
  logger.LogExternalDevice(m_turningEncoder);
}

void SwerveModule::InitSendable(frc::SendableBuilder &builder)
{
  InitSendable(builder, "");
}

void SwerveModule::InitSendable(frc::SendableBuilder &builder, std::string name)
{
  builder.SetSmartDashboardType("SwerveModule");
  builder.SetActuator(true);

  // Prefix for nested objects
  if(name != "") name += "/";

  // Drive Control
  builder.AddDoubleProperty(
      name + "Drive kP", [this] { return m_drivePIDController.GetP(); }, [this](double value) { m_drivePIDController.SetP(value); });
  builder.AddDoubleProperty(
      name + "Drive kI", [this] { return m_drivePIDController.GetI(); }, [this](double value) { m_drivePIDController.SetI(value); });
  builder.AddDoubleProperty(
      name + "Drive kD", [this] { return m_drivePIDController.GetD(); }, [this](double value) { m_drivePIDController.SetD(value); });
  builder.AddDoubleProperty(
      name + "Drive Goal",
      [this] { return units::meters_per_second_t(m_drivePIDController.GetGoal().position).value(); },
      [this](double value) { m_drivePIDController.SetGoal(units::meters_per_second_t(value)); });
  builder.AddDoubleProperty(
      name + "Drive SP",
      [this] { return units::meters_per_second_t(m_drivePIDController.GetSetpoint().position).value(); }, nullptr);
  builder.AddDoubleProperty(
      name + "Velocity", [this] { return units::meters_per_second_t(GetVelocity()).value(); }, nullptr);
  builder.AddDoubleProperty(
      name + "m_driveVolts", [this] { return m_driveVolts.value(); }, nullptr);

  // Angle Control
  builder.AddDoubleProperty(
      name + "Angle kP", [this] { return m_turningPIDController.GetP(); }, [this](double value) { m_turningPIDController.SetP(value); });
  builder.AddDoubleProperty(
      name + "Angle kI", [this] { return m_turningPIDController.GetI(); }, [this](double value) { m_turningPIDController.SetI(value); });
  builder.AddDoubleProperty(
      name + "Angle kD", [this] { return m_turningPIDController.GetD(); }, [this](double value) { m_turningPIDController.SetD(value); });
  builder.AddDoubleProperty(
      name + "Angle Goal",
      [this] { return units::degree_t(m_turningPIDController.GetGoal().position).value(); },
      [this](double value) { m_turningPIDController.SetGoal(units::degree_t(value)); });
  builder.AddDoubleProperty(
      name + "Angle SP",
      [this] { return units::degree_t(m_turningPIDController.GetSetpoint().position).value(); }, nullptr);
  builder.AddDoubleProperty(
      name + "Angle", [this] { return GetAngle().Degrees().value(); }, nullptr);
  builder.AddDoubleProperty(
      name + "m_turnVolts", [this] { return m_turnVolts.value(); }, nullptr);

  // builder.AddDoubleProperty(
  //     "Angle Offset",
  //     [this] { return prefs->GetDouble(m_angleOffsetPref); },
  //     [this](double value) {
  //         prefs->PutDouble(m_angleOffsetPref, value);
  //         m_turningEncoder.ConfigMagnetOffset(value);
  //     });

  // Turning Encoders
  // builder.AddDoubleProperty(
  //     "Encoder Neo", [this] { return frc::Rotation2d(units::radian_t(m_neoEncoder.GetPosition())).Degrees().value(); }, nullptr);
  // builder.AddDoubleProperty(
  //     "Encoder CTRE", [this] { return m_turningEncoder.GetAbsolutePosition(); }, nullptr);

  // Thermal
  builder.AddBooleanProperty(
      name + "Thermal Fault", [this] { return m_faultTermal; }, [this](bool value) { m_faultTermal = value; });
  builder.AddDoubleProperty(
      name + "Drive Temp [C]", [this] { return m_driveMotor.GetTemperature(); }, nullptr);
  builder.AddDoubleProperty(
      name + "Angle Temp [C]", [this] { return m_turningMotor.GetTemperature(); }, nullptr);
}
