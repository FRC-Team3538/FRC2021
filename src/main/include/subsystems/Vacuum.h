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

class Vacuum : public frc::Sendable,
                public frc::SendableHelper<Vacuum>,
                public rj::Loggable
{
public:
    Vacuum();

    void InitSendable(frc::SendableBuilder &builder) override;

    void Set(units::volt_t impeller);

    void Log(UDPLogger &logger);

    void SimPeriodic();
    void Periodic();

    static constexpr auto kMaxImpellerVoltage = 12_V;

private:
    // Hardware
    WPI_TalonFX m_impellerMotor{30};
};
