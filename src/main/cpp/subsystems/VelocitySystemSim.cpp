// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VelocitySystemSim.h"
#include <units/length.h>

#include <wpi/MathExtras.h>

#include "frc/system/plant/LinearSystemId.h"

using namespace frc;
using namespace frc::sim;

VelocitySystemSim::VelocitySystemSim(const LinearSystem<1, 1, 1> &plant,
                                     const DCMotor &gearbox, double gearing, units::meter_t wheelRadius,
                                     const std::array<double, 1> &measurementStdDevs)
    : LinearSystemSim<1, 1, 1>(plant, measurementStdDevs),
      m_gearbox(gearbox),
      m_gearing(gearing),
      m_wheelRadius(wheelRadius) {}

units::meters_per_second_t VelocitySystemSim::GetVelocity() const
{
  return units::meters_per_second_t{GetOutput(0)};
}

units::ampere_t VelocitySystemSim::GetCurrentDraw() const
{
  // I = V / R - omega / (Kv * R)
  // Reductions are greater than 1, so a reduction of 10:1 would mean the motor
  // is spinning 10x faster than the output.

  // rad/sec = (meter/sec) * (radian/meter)
  auto c = 2_rad / (2 * m_wheelRadius * wpi::math::pi);

  return m_gearbox.Current(
             GetVelocity() * c * m_gearing,
             units::volt_t{m_u(0)}) *
         wpi::sgn(m_u(0));
}

void VelocitySystemSim::SetInputVoltage(units::volt_t voltage)
{
  SetInput(frc::MakeMatrix<1, 1>(voltage.to<double>()));
}