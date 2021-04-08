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

#include <utility>

// Vendor Libraries
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"

class Shooter : public frc::Sendable,
                public frc::SendableHelper<Shooter>,
                public rj::Loggable
{
public:
    Shooter();

    void InitSendable(frc::SendableBuilder &builder) override;

    void Set(units::volt_t shooter, units::volt_t gate, units::volt_t hood);

    void SetShooterSpeed(units::radians_per_second_t speed)
    {
        m_targetShooterSpeed = speed;
    }

    void SetHoodAngle(units::radian_t angle)
    {
        m_targetHoodAngle = angle;
    }

    void GotoSetpoint(int setpoint)
    {
        SetShooterSpeed(m_speed_setpoints[setpoint]);
        SetHoodAngle(m_pitch_setpoints[setpoint]);
    }

    units::radians_per_second_t GetShooterSpeed()
    {
        return units::revolutions_per_minute_t{m_shooterMotor1.GetSelectedSensorVelocity() * 600.0 / 2048.0};
    };

    units::radian_t GetHoodAngle()
    {
        return units::radian_t{m_hoodEncoder.GetDistance()};
    };

    void SetGate(units::volt_t gate)
    {
        m_gateMotor.SetVoltage(gate);
    }

    void SetGravityBoost(units::volt_t gboost)
    {
        m_gravityBoost.SetVoltage(gboost);
    }

    void ReZeroHood()
    {
        m_hoodZeroed = false;
    }

    void Log(UDPLogger &logger);

    void SimPeriodic();
    void Periodic();

    static constexpr auto kMaxShooterVoltage = 12_V;
    static constexpr auto kMaxGateVoltage = 12_V;
    static constexpr auto kMaxHoodVoltage = 12_V;
    static constexpr auto kMaxGravityVoltage = 12_V;

    static constexpr auto kMaxShooterVelocity = 2900_rpm;
    static constexpr auto kMaxHoodAngle = 60_deg;

private:
    // Hardware
    WPI_TalonFX m_shooterMotor1{21};
    WPI_TalonFX m_shooterMotor2{22};
    WPI_VictorSPX m_hoodMotor{23};
    WPI_VictorSPX m_gateMotor{24};
    WPI_VictorSPX m_gravityBoost{25};

    frc::Encoder m_hoodEncoder{1, 2, true, frc::CounterBase::EncodingType::k4X};
    frc::DigitalInput m_hoodLowerLimit{4};

    // MAX HOOD ANGLE: 60 deg
    frc::ProfiledPIDController<units::revolutions_per_minute> m_shooterPIDController{0, /*1.21 / 60.,*/ 0, 0, {2000_rad_per_s_sq, 6000_rad_per_s_sq / 1_s}};
    frc::ProfiledPIDController<units::radians> m_hoodPIDController{5.12, 0, 0.0134, {2_rad_per_s, 3_rad_per_s_sq}};

    frc::SimpleMotorFeedforward<units::radians> m_shooterFeedforward{0.73_V, 0.115_V / 60_rpm, 0.026_V / 60_rpm * 1_s};
    frc::SimpleMotorFeedforward<units::radians> m_hoodFeedforward{1.16_V, 4.9_V / 1_rad_per_s, 0.0315_V / 1_rad_per_s_sq};

    units::volt_t m_shooterVolts = 0_V;
    units::volt_t m_hoodVolts = 0_V;

    units::radians_per_second_t m_targetShooterSpeed;
    units::radian_t m_targetHoodAngle;

    units::radian_t hoodPosition = 0_rad;
    bool m_hoodZeroed = false;

    const std::array<units::radians_per_second_t, 5> m_speed_setpoints{
        0_rpm,
        2450_rpm,
        2450_rpm,
        3650_rpm,
        3250_rpm
    };

    const std::array<units::radian_t, 5> m_pitch_setpoints{
        0_deg,
        32.5_deg,
        38_deg,
        49.5_deg, // FRESH BALLS, 47.5* ACTUAL
        47.5_deg
    };
};
