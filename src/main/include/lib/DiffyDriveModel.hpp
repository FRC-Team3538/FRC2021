#pragma once

#include <wpi/math>

#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/current.h>
#include <units/power.h>
#include <units/voltage.h>
#include <units/torque.h>
#include <units/mass.h>
#include <units/length.h>
#include <units/impedance.h>
#include <units/velocity.h>
#include <units/math.h>
#include <units/acceleration.h>
#include <units/force.h>
#include <units/area.h>

#include <lib/MotorModel.hpp>

namespace rj
{

class DiffyDriveModel
{
public:
    units::kilogram_t mass;
    units::inch_t wheel_diameter;
    double gear_ratio;
    units::volt_t open_circuit_voltage;
    units::ohm_t battery_resistance;
    int motor_count;
    units::ampere_t current_limit;
    double friction_coefficient;
    units::inch_t track_width;
    MotorModel motor_model;

    struct DiffyDriveState {
        units::second_t time;
        units::meters_per_second_t wheel_speed;
        units::revolutions_per_minute_t motor_speed;
        units::ampere_t requested_current;
        units::ampere_t drawn_current;
        units::volt_t battery_voltage;
        units::torque::newton_meter_t motor_torque;
        units::newton_t force_on_surface;
        units::meters_per_second_squared_t wheel_acceleration;
        units::newton_t required_downforce;
        units::newton_t required_vacuum;
    };

    DiffyDriveState State(units::meters_per_second_t wheel_speed, units::second_t time = 0_s) const
    {
        auto motor_speed = MotorSpeedForWheelSpeed(wheel_speed);
        auto requested_current = RequestedCurrentAtMotorSpeed(motor_speed);
        auto drawn_current = LimitedCurrent(requested_current);
        auto battery_voltage = BatteryVoltage(drawn_current);
        auto motor_torque = TorqueAtMotor(drawn_current, requested_current, motor_speed, battery_voltage);
        auto force_on_surface = ForceOnSurface(motor_torque);
        auto wheel_accel = WheelAccel(force_on_surface);
        auto downforce_required = RequiredDownforce(force_on_surface);
        auto vacuum_required = RequiredVacuum(downforce_required);
        return {
            time,
            wheel_speed,
            motor_speed,
            requested_current,
            drawn_current,
            battery_voltage,
            motor_torque,
            force_on_surface,
            wheel_accel,
            downforce_required,
            vacuum_required
        };
    }

    DiffyDriveState Init() const
    {
        return State(0_mps);
    }

    DiffyDriveState Iter(DiffyDriveState current_state, units::second_t dt = 0.02_s) const
    {
        auto velocity = current_state.wheel_speed + dt * current_state.wheel_acceleration;
        return State(velocity, current_state.time + dt);
    }

    units::meters_per_second_t MaxVelocity() const
    {
        return motor_model.free_speed * wpi::math::pi * wheel_diameter / gear_ratio / 1_tr;
    }

    units::revolutions_per_minute_t MotorSpeedForWheelSpeed(units::meters_per_second_t wheel_speed) const
    {
        return wheel_speed / (wpi::math::pi * wheel_diameter) * gear_ratio * 1_tr;
    }

    units::ampere_t RequestedCurrentAtMotorSpeed(units::revolutions_per_minute_t motor_speed) const
    {
        auto volt_ratio = open_circuit_voltage
            / (motor_model.test_voltage + units::volt_t(battery_resistance.value() * motor_count));
        return volt_ratio
            * (motor_model.stall_current  
                - (motor_model.stall_current - motor_model.free_current) 
                    * motor_speed / motor_model.free_speed)
            * motor_count;
    }

    units::ampere_t LimitedCurrent(units::ampere_t requested_current) const
    {
        return units::math::min(current_limit * motor_count, requested_current);
    }

    units::volt_t BatteryVoltage(units::ampere_t drawn_current) const
    {
        return open_circuit_voltage - drawn_current * battery_resistance;
    }

    units::torque::newton_meter_t TorqueAtMotor(units::ampere_t drawn_current, units::ampere_t requested_current, units::revolutions_per_minute_t motor_speed, units::volt_t battery_voltage) const
    {
        return motor_model.stall_torque
            * (1 - motor_speed / motor_model.free_speed)
            * battery_voltage / motor_model.test_voltage
            * drawn_current / requested_current;
    }

    units::newton_t ForceOnSurface(units::torque::newton_meter_t torque_at_motor) const
    {
        return torque_at_motor
            * gear_ratio
            * friction_coefficient
            * 2 
            / wheel_diameter
            * motor_count;
    }

    units::meters_per_second_squared_t WheelAccel(units::newton_t force_on_surface) const
    {
        return force_on_surface / mass;
    }

    units::radians_per_second_squared_t MotorAccel(units::meters_per_second_squared_t wheel_accel) const
    {
        return wheel_accel / (wpi::math::pi * wheel_diameter) * gear_ratio * 1_rad;
    }

    units::newton_t RequiredDownforce(units::newton_t forward_force) const
    {
        return forward_force / friction_coefficient;
    }

    units::newton_t RequiredVacuum(units::newton_t required_downforce) const
    {
        return required_downforce - mass * 9.80665_mps_sq;
    }

    units::second_t TimeToCurrentLimit(units::second_t dt) const
    {
        auto t = 0_s;
        auto state = Init();
        while (state.requested_current > state.drawn_current)
        {
            t += dt;
            state = Iter(state, dt);
        }
        return t;
    }

    units::meters_per_second_squared_t MaxAccel() const
    {
        auto state = Init();
        return state.wheel_acceleration;
    }
};

} //namespace rj