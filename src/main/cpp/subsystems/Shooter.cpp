// Copyright (c) FRC3538 - RoboJackets, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/math>
#include <iostream>
#include <cmath>
#include <frc/MathUtil.h>

using namespace frc;

Shooter::Shooter()
{
  m_hoodEncoder.SetDistancePerPulse(1.7473678478167824898284906743e-4);

  m_shooterMotor1.ConfigFactoryDefault();
  m_shooterMotor1.SetInverted(true);
  m_shooterMotor1.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  m_shooterMotor1.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 55, 55, 0.0));

  m_shooterMotor2.ConfigFactoryDefault();
  m_shooterMotor2.SetInverted(false);
  m_shooterMotor2.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  m_shooterMotor2.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 55, 55, 0.0));

  m_gateMotor.ConfigFactoryDefault();
  m_gateMotor.SetInverted(true);
  m_hoodMotor.ConfigFactoryDefault();
  m_hoodMotor.SetInverted(true);
  m_gravityBoost.ConfigFactoryDefault();
  m_gravityBoost.SetInverted(true);
}

void Shooter::Set(units::volt_t shooter, units::volt_t gate, units::volt_t hood)
{
  m_shooterMotor1.SetVoltage(shooter);
  m_shooterMotor2.SetVoltage(shooter);
  m_gateMotor.SetVoltage(gate);
  m_hoodMotor.SetVoltage(hood);
}

void Shooter::SimPeriodic()
{
}

void Shooter::Periodic()
{
  // Hood
  if (!m_hoodZeroed)
  {
    // TODO
    const auto lowHoodReturnSpeed = -5_deg_per_s;
    m_hoodVolts = m_hoodFeedforward.Calculate(lowHoodReturnSpeed);

    // normally closed
    if (!m_hoodLowerLimit.Get())
    {
      m_hoodEncoder.Reset();
      m_hoodZeroed = true;
      m_hoodVolts = 0_V;
    }
  } 
  else
  {
    const auto hoodAngle = GetHoodAngle();

    const auto hoodOutput = m_hoodPIDController.Calculate(hoodAngle, m_targetHoodAngle);
    const auto hoodFf = m_hoodFeedforward.Calculate(m_hoodPIDController.GetSetpoint().velocity);

    m_hoodVolts = units::volt_t{hoodOutput} + hoodFf;
  }

  // Shooter
  const auto currentSpeed = GetShooterSpeed();

  const auto shooterOutput = m_shooterPIDController.Calculate(currentSpeed, m_targetShooterSpeed);
  const auto shooterFf = m_shooterFeedforward.Calculate(m_shooterPIDController.GetSetpoint().position, m_shooterPIDController.GetSetpoint().velocity);

  m_shooterVolts = units::volt_t{shooterOutput} + shooterFf;

  /******************************************************/

  m_hoodMotor.SetVoltage(m_hoodVolts);

  m_shooterMotor1.SetVoltage(m_shooterVolts);
  m_shooterMotor2.SetVoltage(m_shooterVolts);
}

void Shooter::InitSendable(frc::SendableBuilder &builder)
{
  builder.SetSmartDashboardType("Shooter");
  builder.SetActuator(true);

  // Shooter
  builder.AddDoubleProperty(
      "Shooter Velocity [RPM]",
      [this] { return GetShooterSpeed().value() / 2 / 3.14159 * 60; },
      nullptr);
    builder.AddDoubleProperty(
      "Shooter Velocity Target [RPM]",
      [this] { return m_targetShooterSpeed.value() / 2 / 3.14159 * 60; },
      [this](double rpm) { m_targetShooterSpeed = units::revolutions_per_minute_t{rpm}; });
  builder.AddDoubleProperty(
      "Shooter Voltage",
      [this] { return m_shooterMotor1.GetMotorOutputVoltage(); },
      nullptr);
  builder.AddDoubleProperty(
      "Shooter Position Error",
      [this] { return m_shooterPIDController.GetPositionError().value(); },
      nullptr);
  builder.AddDoubleProperty(
      "Shooter Velocity Error",
      [this] { return m_shooterPIDController.GetVelocityError().value(); },
      nullptr);

  // Gate
  // TODO

  // Hood
  builder.AddBooleanProperty(
      "Hood Lower Limit Switch",
      [this] { return m_hoodLowerLimit.Get(); },
      nullptr);
  builder.AddDoubleProperty(
      "Hood Angle [deg]",
      [this] { return GetHoodAngle().value() * 180 / 3.14159; },
      nullptr);
  builder.AddDoubleProperty(
      "Hood Angle [deg]",
      [this] { return GetHoodAngle().value() * 180 / 3.14159; },
      nullptr);
  builder.AddDoubleProperty(
      "Hood Angle Target [deg]",
      [this] { return m_targetHoodAngle.value() * 180 / 3.14159; },
      [this](double angle) { m_targetHoodAngle = units::degree_t{angle}; });
  builder.AddBooleanProperty(
      "Hood Zeroed",
      [this] { return m_hoodZeroed; },
      [this](bool zeroed) { m_hoodZeroed = zeroed; });
  builder.AddDoubleProperty(
      "Hood Position Error [rad]",
      [this] { return m_hoodPIDController.GetPositionError().value(); },
      nullptr);
  builder.AddDoubleProperty(
      "Hood Velocity Error [rad/s]",
      [this] { return m_hoodPIDController.GetVelocityError().value(); },
      nullptr);
  builder.AddDoubleProperty(
      "Hood Encoder Position [rad]",
      [this] { return m_hoodEncoder.GetDistance(); },
      nullptr);

  // Thermal
  builder.AddDoubleProperty(
      "Shooter 1 Temp [C]",
      [this] { return m_shooterMotor1.GetTemperature(); },
      nullptr);
  builder.AddDoubleProperty(
      "Shooter 2 Temp [C]",
      [this] { return m_shooterMotor2.GetTemperature(); },
      nullptr);
}
