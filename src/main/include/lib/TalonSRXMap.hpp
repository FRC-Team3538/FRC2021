#pragma once
#include <ctre/Phoenix.h>

namespace rj {
class TalonSRXMap
{
public:
  int id;
  ctre::phoenix::motorcontrol::InvertType invertType;
  ctre::phoenix::motorcontrol::NeutralMode neutralMode;
};
} // namespace rj