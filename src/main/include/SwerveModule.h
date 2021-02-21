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

#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"

class SwerveModule : public frc::Sendable,
                     public frc::SendableHelper<SwerveModule>
{
public:
    SwerveModule(int driveMotorChannel,
                 int turningMotorChannel,
                 int turningEncoderChannel);
    frc::SwerveModuleState GetState();
    void SetDesiredState(const frc::SwerveModuleState &state);

    void InitSendable(frc::SendableBuilder &builder) override;

private:
    // Configuration
    static constexpr auto kWheelRadius = 3_in;
    static constexpr int kEncoderResolution = 2048;
    static constexpr double kDriveGearboxRatio = 5.25;

    static constexpr auto kMaxLinearAcceleration = 6.0_mps_sq;
    static constexpr auto kMaxLinearJerk = 12.0_mps_sq / 1_s;

    static constexpr auto kMaxAngularVelocity = wpi::math::pi * 1_rad_per_s;
    static constexpr auto kMaxAngularAcceleration = wpi::math::pi * 2_rad_per_s_sq;

    static constexpr auto kDriveScaleFactor =
        (2 * wpi::math::pi * kWheelRadius) / (kDriveGearboxRatio * kEncoderResolution);

    static constexpr auto kTurningMotorVoltageNominal = 12.8_V;

    /********************************************************************************/
    /* Post in slack #controls-software if you change these!                        */
    /********************************************************************************/
    static constexpr auto kTurningMotorCurrentLimit = 20_A;
    static constexpr auto kTurningMotorTemperatureMax = 45_degC;
    /********************************************************************************/

    // Preferences
    frc::Preferences *prefs = frc::Preferences::GetInstance();

    // Hardware
    WPI_TalonFX m_driveMotor;
    rev::CANSparkMax m_turningMotor;
    CANCoder m_turningEncoder;

    // Angle Offset
    std::string m_angleOffsetPref = "SwerveAngleOffset";

    // Control
    frc::ProfiledPIDController<units::meters_per_second> m_drivePIDController{
        1.0,
        0.0,
        0.0,
        {kMaxLinearAcceleration, kMaxLinearJerk}};

    frc::ProfiledPIDController<units::radians> m_turningPIDController{
        1.0,
        0.0,
        0.0,
        {kMaxAngularVelocity, kMaxAngularAcceleration}};

    frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{1_V, 3_V / 1_mps};
    frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{1_V, 0.5_V / 1_rad_per_s};

    // Thermal Limit
    bool m_faultTermal = false;
};
