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
  m_hoodEncoder.SetDistancePerPulse(0.04696044921875);

  m_shooterMotor1.ConfigFactoryDefault();
  m_shooterMotor1.SetInverted(true);
  m_shooterMotor1.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  m_shooterMotor1.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 55, 55, 0.0));

  m_shooterMotor2.ConfigFactoryDefault();
  m_shooterMotor2.SetInverted(false);
  m_shooterMotor2.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  m_shooterMotor2.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 55, 55, 0.0));

  m_gateMotor.ConfigFactoryDefault();
  m_hoodMotor.ConfigFactoryDefault();
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

void Shooter::Log(UDPLogger &logger)
{
  logger.LogExternalDevice(m_shooterMotor1);
  logger.LogExternalDevice(m_shooterMotor2);
  logger.LogExternalDevice(m_gateMotor);
  logger.LogExternalDevice(m_hoodMotor);
}

void Shooter::InitSendable(frc::SendableBuilder &builder)
{
  builder.SetSmartDashboardType("Shooter");
  builder.SetActuator(true);

  // Shooter
  builder.AddDoubleProperty(
    "Shooter Velocity [RPM@Motor]", 
    [this] { return GetShooterSpeed().value(); }, 
    [this](double rpm) {m_targetShooterSpeed = units::revolutions_per_minute_t{rpm};});
  builder.AddDoubleProperty(
    "Shooter Voltage", 
    [this] { return m_shooterMotor1.GetMotorOutputVoltage(); }, 
    nullptr);

  // Gate
  // TODO

  // Hood
  builder.AddBooleanProperty(
    "Hood Lower Limit Switch", 
    [this] { return m_hoodLowerLimit.Get(); }, 
    nullptr);
  builder.AddDoubleProperty(
    "Hood Angle", 
    [this] {return GetHoodAngle().value();}, 
    [this](double angle) {m_targetHoodAngle = units::degree_t{angle};});
  builder.AddBooleanProperty(
    "Hood Zeroed", 
    [this] { return m_hoodZeroed;}, 
    [this](bool zeroed) {m_hoodZeroed = zeroed;});

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
