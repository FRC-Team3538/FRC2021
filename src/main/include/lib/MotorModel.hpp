#pragma once

#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/power.h>
#include <units/voltage.h>
#include <units/torque.h>

namespace rj
{

class MotorModel
{
public:
  units::revolutions_per_minute_t free_speed;
  units::ampere_t free_current;
  units::watt_t max_power;
  units::torque::newton_meter_t stall_torque;
  units::ampere_t stall_current;
  units::volt_t test_voltage;
};

} //namespace rj