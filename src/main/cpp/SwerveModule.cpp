// Copyright (c) FRC3538 - RoboJackets, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
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

    // Drive
    const auto driveOutput = m_drivePIDController.Calculate(
        GetState().speed,
        state.speed);

    const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

    m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);

    // Angle
    const auto turnOutput = m_turningPIDController.Calculate(
        GetState().angle.Radians(), state.angle.Radians());

    const auto turnFeedforward = m_turnFeedforward.Calculate(
        m_turningPIDController.GetSetpoint().velocity);

    m_turningMotor.SetVoltage(units::volt_t{turnOutput} + turnFeedforward);
}

void SwerveModule::InitSendable(frc::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("SwerveModule");
    builder.SetActuator(true);

    // Drive Control
    builder.AddDoubleProperty(
        "Drive P", [this] { return m_drivePIDController.GetP(); }, [this](double value) { m_drivePIDController.SetP(value); });
    builder.AddDoubleProperty(
        "Drive I", [this] { return m_drivePIDController.GetI(); }, [this](double value) { m_drivePIDController.SetI(value); });
    builder.AddDoubleProperty(
        "Drive D", [this] { return m_drivePIDController.GetD(); }, [this](double value) { m_drivePIDController.SetD(value); });
    builder.AddDoubleProperty(
        "Drive Goal",
        [this] { return m_drivePIDController.GetGoal().velocity.value(); },
        [this](double value) { m_drivePIDController.SetGoal(units::meters_per_second_t(value)); });
    builder.AddDoubleProperty(
        "Velocity", [this] { return GetState().speed.value(); }, nullptr);

    // Angle Control
    builder.AddDoubleProperty(
        "Angle P", [this] { return m_turningPIDController.GetP(); }, [this](double value) { m_turningPIDController.SetP(value); });
    builder.AddDoubleProperty(
        "Angle I", [this] { return m_turningPIDController.GetI(); }, [this](double value) { m_turningPIDController.SetI(value); });
    builder.AddDoubleProperty(
        "Angle D", [this] { return m_turningPIDController.GetD(); }, [this](double value) { m_turningPIDController.SetD(value); });
    builder.AddDoubleProperty(
        "Angle Goal",
        [this] { return units::degree_t(m_turningPIDController.GetGoal().position).value(); },
        [this](double value) { m_turningPIDController.SetGoal(units::degree_t(value)); });
    builder.AddDoubleProperty(
        "Angle", [this] { return GetState().angle.Degrees().value(); }, nullptr);

    builder.AddDoubleProperty(
        "Angle Offset",
        [this] { return m_angleOffset.Degrees().value(); },
        [this](double value) {
            prefs->PutDouble(m_angleOffsetPref, value);
            m_angleOffset = units::degree_t(value);
        });

    // Thermal
    builder.AddBooleanProperty(
        "Thermal Fault", [this] { return m_faultTermal; }, [this](bool value) { m_faultTermal = value; });
    builder.AddDoubleProperty(
        "Drive Temp [C]", [this] { return m_driveMotor.GetTemperature(); }, nullptr);
    builder.AddDoubleProperty(
        "Angle Temp [C]", [this] { return m_turningMotor.GetMotorTemperature(); }, nullptr);
}
