#pragma once
#include "lib/TalonSRXMap.hpp"
#include "lib/VictorSPXMap.hpp"
#include <ctre/Phoenix.h>

namespace rj {
inline void
ConfigureWPI_TalonSRX(
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX& controller,
  rj::TalonSRXMap& map,
  ctre::phoenix::motorcontrol::can::TalonSRXConfiguration& config)
{
  controller.ConfigFactoryDefault();
  controller.ConfigAllSettings(config);
  controller.SetInverted(map.invertType);
  controller.SetNeutralMode(map.neutralMode);
}

inline void
ConfigureWPI_VictorSPX(
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX& controller,
  rj::VictorSPXMap& map,
  ctre::phoenix::motorcontrol::can::VictorSPXConfiguration& config)
{
  controller.ConfigFactoryDefault();
  controller.ConfigAllSettings(config);
  controller.SetInverted(map.invertType);
  controller.SetNeutralMode(map.neutralMode);
}
} // namespace rj
