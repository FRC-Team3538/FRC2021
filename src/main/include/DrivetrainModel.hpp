#pragma once

#include <lib/DiffyDriveTrajectoryConstraint.hpp>

// Source: motors.vex.com
constexpr rj::MotorModel falcon{
    .free_speed = 6380_rpm,
    .free_current = 1.5_A,
    .max_power = 783_W,
    .stall_torque = 4.69_Nm,
    .stall_current = 257_A,
    .test_voltage = 12_V};

// SET THESE BEFORE USE
constexpr rj::DiffyDriveModel grasshopper{
    .mass = 41.95_lb,
    .wheel_diameter = 4.25_in,
    .gear_ratio = 5.95,
    // This will be different for every run - TODO: set this in AutonomousInit()
    .open_circuit_voltage = 12.8_V,
    // This will be different for every battery, but there's no good way to get this on-bot.
    // Maybe beak it and shuffleboard?
    .battery_resistance = 0.03_Ohm,
    .motor_count = 6,
    // Per Motor
    .current_limit = 55_A,
    .friction_coefficient = 0.9,
    .track_width = .6488_m, //.579
    .motor_model = falcon};