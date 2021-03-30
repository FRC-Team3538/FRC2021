// Copyright (c) FRC3538 - RoboJackets, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Vacuum.h"

#include <frc/geometry/Rotation2d.h>
#include <wpi/math>
#include <iostream>
#include <cmath>
#include <frc/MathUtil.h>

using namespace frc;

Vacuum::Vacuum()
{
  m_impellerMotor.ConfigFactoryDefault();
  m_impellerMotor.SetInverted(false);
  m_impellerMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
  m_impellerMotor.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 55, 55, 0.0));
}

void Vacuum::SimPeriodic()
{
}

void Vacuum::Periodic()
{

}

void Vacuum::Set(units::volt_t volts)
{
    m_impellerMotor.SetVoltage(volts);
}

void Vacuum::Log(UDPLogger &logger)
{
  logger.LogExternalDevice(m_impellerMotor);
}

void Vacuum::InitSendable(frc::SendableBuilder &builder)
{
  builder.SetSmartDashboardType("Vacuum");
  builder.SetActuator(true);

  // Shooter
  builder.AddDoubleProperty(
      "Shooter Velocity [RPM]",
      [this] { return m_impellerMotor.GetSelectedSensorVelocity() / 2048 / 10 / 60; },
      nullptr);
  builder.AddDoubleProperty(
      "Shooter Voltage",
      [this] { return m_impellerMotor.GetMotorOutputVoltage(); },
      nullptr);

  builder.AddDoubleProperty(
      "Impeller Temp [C]",
      [this] { return m_impellerMotor.GetTemperature(); },
      nullptr);
}
