// Copyright (c) FRC3538 - RoboJackets, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/AnalogGyro.h>
#include <wpi/math>

#include "SwerveModule.h"
#include "adi/ADIS16470_IMU.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain
{
public:
    Drivetrain();

    void Drive(units::meters_per_second_t xSpeed,
               units::meters_per_second_t ySpeed,
               units::radians_per_second_t rot,
               bool fieldRelative = true);
    void UpdateOdometry();

    frc::Rotation2d GetYaw();

    void Log();

    static constexpr auto kMaxSpeed = 10_fps;
    static constexpr auto kMaxAngularSpeed = wpi::math::pi * 1_rad_per_s;

private:
    // Configuration
    static constexpr auto m_dist = 15.75_in / 2;
    frc::Translation2d m_frontLeftLocation{+m_dist, +m_dist};
    frc::Translation2d m_frontRightLocation{+m_dist, -m_dist};
    frc::Translation2d m_backLeftLocation{-m_dist, +m_dist};
    frc::Translation2d m_backRightLocation{-m_dist, -m_dist};

    // Swerve Modules
    SwerveModule m_frontLeft{1, 2, 3};
    SwerveModule m_frontRight{1, 5, 6};
    SwerveModule m_backLeft{6, 8, 9};
    SwerveModule m_backRight{10, 11, 12};

    // Gyro / IMU
#ifdef __FRC_ROBORIO__
    frc::ADIS16470_IMU m_imu{
        frc::ADIS16470_IMU::IMUAxis::kZ,
        frc::SPI::Port::kMXP,
        frc::ADIS16470CalibrationTime::_512ms};
#else
    // The ADI gyro is not simulator compatible on linux
    frc::AnalogGyro m_gyro{0};
#endif

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
