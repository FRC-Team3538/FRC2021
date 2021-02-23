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
    frc::Rotation2d GetAngle();

    void InitSendable(frc::SendableBuilder &builder) override;

    void Set(double drive, double azimuth);

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
    // CANCoder m_turningEncoder;
    // encoder returns position in turns, so no need to use the internal ticks for added precision
    // here we convert motor turns to wheel radians
    rev::CANEncoder m_neoEncoder = m_turningMotor.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42.0 * kEncoderGearboxRatio); 
    rev::CANPIDController m_neoPIDController = m_turningMotor.GetPIDController();

    // Angle Offset
    std::string m_angleOffsetPref = "SwerveAngleOffset";

    // Control
    // TODO: Determine these values for actual-weight
    frc::ProfiledPIDController<units::meters_per_second> m_drivePIDController{
        1.08,
        0.0,
        0.0,
        {kMaxLinearAcceleration, kMaxLinearJerk}};

    frc::ProfiledPIDController<units::radians> m_turningPIDController{
        13.5,
        1.54,
        0.0,
        {kMaxAngularVelocity, kMaxAngularAcceleration}};

    frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{0.671_V, 2.32_V / 1_mps, 0.0452_V / 1_mps_sq};
    frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{0.196_V, 0.534_V / 1_rad_per_s, 0.0348_V / 1_rad_per_s_sq};

    // Thermal Limit
    bool m_faultTermal = false;
};
