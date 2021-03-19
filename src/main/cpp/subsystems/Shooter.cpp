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
  m_shooterMotor1.ConfigFactoryDefault();
  m_shooterMotor1.SetInverted(true);
  m_shooterMotor1.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  m_shooterMotor1.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 55, 55, 0.0));

  m_shooterMotor2.ConfigFactoryDefault();
  m_shooterMotor2.SetInverted(false);
  m_shooterMotor2.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  m_shooterMotor2.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 55, 55, 0.0));

  m_shooterMotor2.Follow(m_shooterMotor1);

  m_gateMotor.ConfigFactoryDefault();
  m_hoodMotor.ConfigFactoryDefault();
}

void Shooter::Set(units::volt_t shooter, units::volt_t gate, units::volt_t hood)
{
    m_shooterMotor1.SetVoltage(shooter);
    m_gateMotor.SetVoltage(gate);
    m_hoodMotor.SetVoltage(hood);
}

void Shooter::SimPeriodic()
{
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
  builder.AddDoubleProperty("Shooter Velocity [RPM@Motor]", [this] { return m_shooterMotor1.GetSelectedSensorVelocity(); }, nullptr);

  // Gate
  // TODO

  // Hood
  // TODO

  // Thermal
  builder.AddDoubleProperty(
      "Shooter 1 Temp [C]", [this] { return m_shooterMotor1.GetTemperature(); }, nullptr);
  builder.AddDoubleProperty(
      "Shooter 2 Temp [C]", [this] { return m_shooterMotor2.GetTemperature(); }, nullptr);
}
