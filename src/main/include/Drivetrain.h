// Copyright (c) FRC3538 - RoboJackets, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <wpi/math>

#include "SwerveModule.h"
#include "adi/ADIS16470_IMU.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain
{
public:
    Drivetrain() { m_imu.Reset(); }

    void Drive(units::meters_per_second_t xSpeed,
               units::meters_per_second_t ySpeed,
               units::radians_per_second_t rot,
               bool fieldRelative = true);
    void UpdateOdometry();

    static constexpr auto kMaxSpeed = 3.0_mps;
    static constexpr auto kMaxAngularSpeed = wpi::math::pi * 1_rad_per_s;

private:
    // Configuration
    frc::Translation2d m_frontLeftLocation{+8_in, +8_in};
    frc::Translation2d m_frontRightLocation{+8_in, -8_in};
    frc::Translation2d m_backLeftLocation{-8_in, +8_in};
    frc::Translation2d m_backRightLocation{-8_in, -8_in};

    // Swerve Modules
    SwerveModule m_frontLeft{1, 1, 0};
    SwerveModule m_frontRight{3, 4, 1};
    SwerveModule m_backLeft{5, 6, 2};
    SwerveModule m_backRight{7, 8, 3};

    // Gyro / IMU
    frc::ADIS16470_IMU m_imu{
        frc::ADIS16470_IMU::IMUAxis::kZ, 
        frc::SPI::Port::kMXP, 
        frc::ADIS16470CalibrationTime::_512ms
    };

    // Odometry
    frc::SwerveDriveKinematics<4> m_kinematics{
        m_frontLeftLocation,
        m_frontRightLocation,
        m_backLeftLocation,
        m_backRightLocation};

    frc::SwerveDriveOdometry<4> m_poseEstimator{
        m_kinematics,
        frc::Rotation2d(),
        frc::Pose2d()};
};
