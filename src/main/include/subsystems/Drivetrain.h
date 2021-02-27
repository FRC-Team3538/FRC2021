// Copyright (c) FRC3538 - RoboJackets, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/AnalogGyro.h>
#include <wpi/math>

#include "subsystems/SwerveModule.h"
#include "adi/ADIS16470_IMU.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain: public rj::Loggable
{
public:
    Drivetrain();

    void Drive(units::meters_per_second_t xSpeed,
               units::meters_per_second_t ySpeed,
               units::radians_per_second_t rot,
               bool fieldRelative = true);
    void UpdateOdometry();

    frc::Rotation2d GetYaw();

    void Log(UDPLogger &logger)
    {
      m_frontLeft.Log(logger);
      m_frontRight.Log(logger);
      m_backLeft.Log(logger);
      m_backRight.Log(logger);
#ifdef __FRC_ROBORIO__
      logger.LogExternalDevice(m_imu);
#endif // __FRC_ROBORIO__
    }

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
    SwerveModule m_frontLeft{1, 2, 3, m_frontLeftConfig};
    SwerveModule m_frontRight{4, 5, 6, m_frontRightConfig};
    SwerveModule m_backLeft{7, 8, 9, m_backLeftConfig};
    SwerveModule m_backRight{10, 11, 12, m_backRightConfig};

    // Gyro / IMU
#ifdef __FRC_ROBORIO__
    frc::ADIS16470_IMU m_imu{
        frc::ADIS16470_IMU::IMUAxis::kZ,
        frc::SPI::Port::kOnboardCS0,
        frc::ADIS16470CalibrationTime::_4s};
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

    // Control
    static constexpr auto kMaxModuleLinearAcceleration = 6.0_mps_sq;
    static constexpr auto kMaxModuleLinearJerk = 12.0_mps_sq / 1_s;

    static constexpr auto kMaxModuleAngularVelocity = wpi::math::pi * 4_rad_per_s;
    static constexpr auto kMaxModuleAngularAcceleration = wpi::math::pi * 8_rad_per_s_sq;

    const SwerveModuleConfig m_frontLeftConfig{
        145.547_deg,
        {
            3.26,
            0.0,
            0.0,
            {kMaxModuleLinearAcceleration, kMaxModuleLinearJerk}},
        {
            9.73,
            0.0,
            3.9,
            {kMaxModuleAngularVelocity, kMaxModuleAngularAcceleration}},
        {0.668_V, 2.39_V / 1_mps, 0.143_V / 1_mps_sq},
        {0.976_V, 3.61_V / 1_rad_per_s, 0.257_V / 1_rad_per_s_sq}
    };
    
    const SwerveModuleConfig m_frontRightConfig{
        -60.469_deg,
        {
            3.26,
            0.0,
            0.0,
            {kMaxModuleLinearAcceleration, kMaxModuleLinearJerk}},
        {
            9.61,
            0.0,
            3.72,
            {kMaxModuleAngularVelocity, kMaxModuleAngularAcceleration}},
        {0.668_V, 2.39_V / 1_mps, 0.143_V / 1_mps_sq},
        {0.656_V, 3.7_V / 1_rad_per_s, 0.237_V / 1_rad_per_s_sq}
    };

    const SwerveModuleConfig m_backLeftConfig{
        98.789_deg,
        {
            3.26,
            0.0,
            0.0,
            {kMaxModuleLinearAcceleration, kMaxModuleLinearJerk}},
        {
            9.34,
            0.0,
            3.54,
            {kMaxModuleAngularVelocity, kMaxModuleAngularAcceleration}},
        {0.668_V, 2.39_V / 1_mps, 0.143_V / 1_mps_sq},
        {0.777_V, 3.62_V / 1_rad_per_s, 0.209_V / 1_rad_per_s_sq}
    };

    const SwerveModuleConfig m_backRightConfig{
        126.387_deg,
        {
            3.26,
            0.0,
            0.0,
            {kMaxModuleLinearAcceleration, kMaxModuleLinearJerk}},
        {
            9.77,
            0.0,
            3.98,
            {kMaxModuleAngularVelocity, kMaxModuleAngularAcceleration}},
        {0.668_V, 2.39_V / 1_mps, 0.143_V / 1_mps_sq},
        {0.764_V, 3.54_V / 1_rad_per_s, 0.264_V / 1_rad_per_s_sq}
    };


};
