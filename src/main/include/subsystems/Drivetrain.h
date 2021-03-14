// Copyright (c) FRC3538 - RoboJackets, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/AnalogGyro.h>
#include <wpi/math>
#include <frc/smartdashboard/Field2d.h>

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
    void ResetYaw();
    void Log(UDPLogger &logger);
    void SimPeriodic();


    static constexpr auto kMaxSpeed = 16_fps;
    // about 2.5 turns per second
    // calculated theoretically to 2.743821154 turns/sec
    static constexpr auto kMaxAngularSpeed = 16_rad_per_s;

private:
    // Configuration
    static constexpr auto m_dist = 15.75_in / 2;
    frc::Translation2d m_frontLeftLocation{+m_dist, -m_dist};
    frc::Translation2d m_frontRightLocation{+m_dist, +m_dist};
    frc::Translation2d m_backLeftLocation{-m_dist, -m_dist};
    frc::Translation2d m_backRightLocation{-m_dist, +m_dist};

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

    frc::Field2d m_fieldDisplay;

    units::radian_t m_theta = 0_rad;

    // Control
    static constexpr auto kMaxModuleLinearAcceleration = 80.0_mps_sq;
    static constexpr auto kMaxModuleLinearJerk = 200.0_mps_sq / 1_s;

    static constexpr auto kMaxModuleAngularVelocity = 18_rad_per_s;
    static constexpr auto kMaxModuleAngularAcceleration = 200_rad_per_s_sq;

    static constexpr SwerveModuleConfig m_frontLeftConfig{
        units::degree_t(-150.205), // units::degree_t(-145.107),
        {
            3.26,
            0.0,
            0.0,
            kMaxModuleLinearAcceleration, 
            kMaxModuleLinearJerk
        },
        {
            1.08,
            0.0,
            0.175,
            kMaxModuleAngularVelocity, 
            kMaxModuleAngularAcceleration
        },
        {
            0.668_V, 
            2.39_V / 1_mps, 
            0.143_V / 1_mps_sq
        },
        {
            0.983_V, 
            0.588_V / 1_rad_per_s, 
            0.0496_V / 1_rad_per_s_sq
        }
    };
    
    static constexpr SwerveModuleConfig m_frontRightConfig{
        units::degree_t(-125.684), // units::degree_t(-128.320),
        {
            3.26,
            0.0,
            0.0,
            kMaxModuleLinearAcceleration, 
            kMaxModuleLinearJerk
        },
        {
            2.83,
            0.0,
            0.0385,
            kMaxModuleAngularVelocity, 
            kMaxModuleAngularAcceleration
        },
        {
            0.668_V, 
            2.39_V / 1_mps, 
            0.143_V / 1_mps_sq
        },
        {
            0.537_V, 
            0.543_V / 1_rad_per_s, 
            0.0204_V / 1_rad_per_s_sq
        }
    };

    static constexpr SwerveModuleConfig m_backLeftConfig{
        101.426_deg, // 97.822_deg,
        {
            3.26,
            0.0,
            0.0,
            kMaxModuleLinearAcceleration, 
            kMaxModuleLinearJerk
        },
        {
            1.64,
            0.0,
            0.0988,
            kMaxModuleAngularVelocity, 
            kMaxModuleAngularAcceleration
        },
        {
            0.668_V, 
            2.39_V / 1_mps, 
            0.143_V / 1_mps_sq
        },
        {
            0.698_V, 
            0.6_V / 1_rad_per_s, 
            0.0387_V / 1_rad_per_s_sq
        }
    };

    static constexpr SwerveModuleConfig m_backRightConfig{
        126.650_deg, // 125.771_deg,
        {
            3.26,
            0.0,
            0.0,
            kMaxModuleLinearAcceleration, 
            kMaxModuleLinearJerk
        },
        {
            0.943,
            0.0,
            0.174,
            kMaxModuleAngularVelocity, 
            kMaxModuleAngularAcceleration
        },
        {
            0.668_V, 
            2.39_V / 1_mps, 
            0.143_V / 1_mps_sq
        },
        {
            1.19_V, 
            0.575_V / 1_rad_per_s, 
            0.0496_V / 1_rad_per_s_sq
        }
    };


};
