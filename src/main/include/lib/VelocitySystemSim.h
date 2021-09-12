// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/velocity.h>
#include <units/length.h>

#include "frc/simulation/LinearSystemSim.h"
#include "frc/system/LinearSystem.h"
#include "frc/system/plant/DCMotor.h"

namespace frc::sim
{
  /**
 * Represents a simulated Velocity mechanism.
 */
  class VelocitySystemSim : public LinearSystemSim<1, 1, 1>
  {
  public:
    /**
   * Creates a simulated VelocitySystem mechanism.
   *
   * @param plant              The linear system representing the VelocitySystem.
   * @param gearbox            The type of and number of motors in the
   *                           gearbox.
   * @param gearing            The gearing of the system (numbers greater than
   *                           1 represent reductions).
   * @param wheelRadius        Radius of the drive wheel
   * @param measurementStdDevs The standard deviation of the measurement noise.
   */
    VelocitySystemSim(
        const LinearSystem<1, 1, 1> &plant,
        const DCMotor &gearbox,
        double gearing,
        units::meter_t wheelRadius,
        const std::array<double, 1> &measurementStdDevs = {0.0});

    /**
   * Returns the velocity.
   *
   * @return The velocity.
   */
    units::meters_per_second_t GetVelocity() const;

    /**
   * Returns the current draw.
   *
   * @return The current draw.
   */
    units::ampere_t GetCurrentDraw() const override;

    /**
   * Sets the input voltage for the flywheel.
   *
   * @param voltage The input voltage.
   */
    void SetInputVoltage(units::volt_t voltage);

  private:
    DCMotor m_gearbox;
    double m_gearing;
    units::meter_t m_wheelRadius;
  };
} // namespace frc::sim
