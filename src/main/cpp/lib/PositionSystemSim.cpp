// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "lib/PositionSystemSim.h"
#include <units/length.h>

#include <wpi/MathExtras.h>

#include "frc/system/plant/LinearSystemId.h"

using namespace frc;
using namespace frc::sim;

PositionSystemSim::PositionSystemSim(const LinearSystem<2, 1, 1> &plant,
                                     const DCMotor &gearbox,
                                     double gearing,
                                     const std::array<double, 1> &measurementStdDevs)
    : LinearSystemSim<2, 1, 1>(plant, measurementStdDevs),
      m_gearbox(gearbox),
      m_gearing(gearing)
{
}

units::radian_t PositionSystemSim::GetPosition() const
{
  return units::radian_t{GetOutput(0)};
}

units::radians_per_second_t PositionSystemSim::GetVelocity() const
{
  return units::radians_per_second_t{GetOutput(1)};
}

units::ampere_t PositionSystemSim::GetCurrentDraw() const
{
  // I = V / R - omega / (Kv * R)
  // Reductions are greater than 1, so a reduction of 10:1 would mean the motor
  // is spinning 10x faster than the output.
  return m_gearbox.Current(
             GetVelocity() * m_gearing,
             units::volt_t{m_u(0)}) *
         wpi::sgn(m_u(0));
}

void PositionSystemSim::SetInputVoltage(units::volt_t voltage)
{
  SetInput(frc::MakeMatrix<1, 1>(voltage.to<double>()));
}