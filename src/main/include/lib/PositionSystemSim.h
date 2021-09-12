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
 * Represents a simulated Position mechanism.
 */
  class PositionSystemSim : public LinearSystemSim<2, 1, 1>
  {
  public:
    /**
   * Creates a simulated PositionSystem mechanism.
   *
   * @param plant              The linear system representing the PositionSystem.
   * @param gearbox            The type of and number of motors in the gearbox.
   * @param gearing            The gearing of the system (numbers greater than
   *                           1 represent reductions).
   * @param measurementStdDevs The standard deviation of the measurement noise.
   */
    PositionSystemSim(
        const LinearSystem<2, 1, 1> &plant,
        const DCMotor &gearbox,
        double gearing,
        const std::array<double, 1> &measurementStdDevs = {0.0});

    /**
   * Returns the Position.
   *
   * @return The position.
   */
    units::radian_t GetPosition() const;

    /**
   * Returns the velocity.
   *
   * @return The velocity.
   */
    units::radians_per_second_t GetVelocity() const;

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
  };
} // namespace frc::sim
