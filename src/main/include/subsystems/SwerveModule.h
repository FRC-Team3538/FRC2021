// Copyright (c) FRC3538 - RoboJackets, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AnalogEncoder.h>
#include <frc/AnalogInput.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/Timer.h>
#include <frc/Preferences.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/velocity.h>
#include <units/current.h>
#include <units/acceleration.h>
#include <units/voltage.h>
#include <units/temperature.h>
#include <units/time.h>
#include <wpi/math>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Sendable.h>
#include <frc/smartdashboard/SendableBuilder.h>
#include <frc/smartdashboard/SendableHelper.h>
#include <lib/Loggable.hpp>

#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"

struct SwerveModuleDrivePIDConfig
{
    const double kP;
    const double kI;
    const double kD;
    const units::meters_per_second_squared_t max_acceleration;
    const decltype(1_mps_sq / 1_s) max_jerk;
};

struct SwerveModuleDriveFFConfig
{
    const units::volt_t kS;
    const decltype(1_V / 1_mps) kV;
    const decltype(1_V / 1_mps_sq) kA;
};

struct SwerveModuleTurnPIDConfig
{
    const double kP;
    const double kI;
    const double kD;
    const units::radians_per_second_t max_angular_velocity;
    const units::radians_per_second_squared_t max_angular_acceleration;
};

struct SwerveModuleTurnFFConfig
{
    const units::volt_t kS;
    const decltype(1_V / 1_rad_per_s) kV;
    const decltype(1_V / 1_rad_per_s_sq) kA;
};

struct SwerveModuleConfig
{
    const units::degree_t angleOffset;
    const SwerveModuleDrivePIDConfig drivePID;
    const SwerveModuleTurnPIDConfig turningPID;
    const SwerveModuleDriveFFConfig driveFf;
    const SwerveModuleTurnFFConfig turnFf;
};

class SwerveModule : public frc::Sendable,
                     public frc::SendableHelper<SwerveModule>,
                     public rj::Loggable
{
public:
    SwerveModule(int driveMotorChannel,
                 int turningMotorChannel,
                 int turningEncoderChannel,
                 SwerveModuleConfig config);
    frc::SwerveModuleState GetState();
    void SetDesiredState(const frc::SwerveModuleState &state);
    frc::Rotation2d GetAngle();

    void InitSendable(frc::SendableBuilder &builder) override;

    void Set(double drive, double azimuth);

    void Log(UDPLogger &logger)
    {
      logger.LogExternalDevice(m_driveMotor);
      logger.LogExternalDevice(m_turningMotor);
      logger.LogExternalDevice(m_turningEncoder);
    }

private:
    // Configuration
    static constexpr auto kWheelRadius = 1.5_in;
    static constexpr int kEncoderResolution = 2048;
    static constexpr double kDriveGearboxRatio = 5.25;
    static constexpr double kEncoderGearboxRatio = 5.33 * 2.89 * 3.61;

    static constexpr auto kMaxLinearAcceleration = 6.0_mps_sq;
    static constexpr auto kMaxLinearJerk = 12.0_mps_sq / 1_s;

    static constexpr auto kMaxAngularVelocity = wpi::math::pi * 4_rad_per_s;
    static constexpr auto kMaxAngularAcceleration = wpi::math::pi * 8_rad_per_s_sq;

    static constexpr auto kDriveScaleFactor =
        (2 * wpi::math::pi * kWheelRadius) / (kDriveGearboxRatio * kEncoderResolution);
    
    static constexpr auto kTurningMotorVoltageNominal = 12.8_V;

    /********************************************************************************/
    /* Post in slack #controls-software if you change these!                        */
    /********************************************************************************/
    static constexpr auto kTurningMotorCurrentLimit = 20_A;
    static constexpr auto kTurningMotorTemperatureMax = units::celsius_t(45);
    /********************************************************************************/

    // Preferences
    frc::Preferences *prefs = frc::Preferences::GetInstance();

    // Hardware
    WPI_TalonFX m_driveMotor;
    rev::CANSparkMax m_turningMotor;
    CANCoder m_turningEncoder;
    // encoder returns position in turns, so no need to use the internal ticks for added precision
    // here we convert motor turns to wheel radians
    rev::CANEncoder m_neoEncoder = m_turningMotor.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor); 
    rev::CANPIDController m_neoPIDController = m_turningMotor.GetPIDController();

    // Angle Offset
    // std::string m_angleOffsetPref = "SwerveAngleOffset";

    // Control
    frc::ProfiledPIDController<units::meters_per_second> m_drivePIDController;
    frc::ProfiledPIDController<units::radians> m_turningPIDController;

    frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward;
    frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward;

    // Thermal Limit
    bool m_faultTermal = false;
};
