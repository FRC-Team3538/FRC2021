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

#include <frc/smartdashboard/Sendable.h>
#include <frc/smartdashboard/SendableBuilder.h>
#include <frc/smartdashboard/SendableHelper.h>

#include <frc/controller/HolonomicDriveController.h>

#include "subsystems/SwerveModule.h"
#include "adi/ADIS16470_IMU.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain : public frc::Sendable,
                   public frc::SendableHelper<SwerveModule>,
                   public rj::Loggable
{
public:
    Drivetrain();

    void Drive(units::meters_per_second_t xSpeed,
               units::meters_per_second_t ySpeed,
               units::radians_per_second_t rot,
               bool fieldRelative = true);
    void Drive(frc::Trajectory trajectory, units::second_t timepoint, units::radian_t yaw = 0_rad);
    void UpdateOdometry();
    frc::Rotation2d GetYaw();
    void ResetYaw();
    void Log(UDPLogger &logger);
    void SimPeriodic();
    void InitSendable(frc::SendableBuilder &builder) override;
    void ResetOdometry(const frc::Pose2d& pose);

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

    bool m_fieldRelative;

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
    
    frc::ChassisSpeeds m_robotVelocity;

    frc::Field2d m_fieldDisplay;

    units::radian_t m_theta = 0_rad;

    // Control
    frc::ChassisSpeeds m_command;

    static constexpr auto kMaxModuleLinearAcceleration = 80.0_mps_sq;
    static constexpr auto kMaxModuleLinearJerk = 200.0_mps_sq / 1_s;

    static constexpr auto kMaxModuleAngularVelocity = 18_rad_per_s;
    static constexpr auto kMaxModuleAngularAcceleration = 200_rad_per_s_sq;

    static constexpr SwerveModuleConfig m_frontLeftConfig{
        units::degree_t(-150.205),
        {
            3.26,
            0.0,
            0.0,
            kMaxModuleLinearAcceleration,
            kMaxModuleLinearJerk},
        {1.08,
         0.0,
         0.85,
         kMaxModuleAngularVelocity,
         kMaxModuleAngularAcceleration},
        {0.668_V,
         2.39_V / 1_mps,
         0.143_V / 1_mps_sq},
        {0.983_V,
         0.588_V / 1_rad_per_s,
         0.0496_V / 1_rad_per_s_sq}};

    static constexpr SwerveModuleConfig m_frontRightConfig{
        units::degree_t(-125.684),
        {
            3.26,
            0.0,
            0.0,
            kMaxModuleLinearAcceleration,
            kMaxModuleLinearJerk},
        {2.83,
         0.0,
         0.85,
         kMaxModuleAngularVelocity,
         kMaxModuleAngularAcceleration},
        {0.668_V,
         2.39_V / 1_mps,
         0.143_V / 1_mps_sq},
        {0.537_V,
         0.543_V / 1_rad_per_s,
         0.0204_V / 1_rad_per_s_sq}};

    static constexpr SwerveModuleConfig m_backLeftConfig{
        101.426_deg,
        {
            3.26,
            0.0,
            0.0,
            kMaxModuleLinearAcceleration,
            kMaxModuleLinearJerk},
        {1.64,
         0.0,
         0.85,
         kMaxModuleAngularVelocity,
         kMaxModuleAngularAcceleration},
        {0.668_V,
         2.39_V / 1_mps,
         0.143_V / 1_mps_sq},
        {0.698_V,
         0.6_V / 1_rad_per_s,
         0.0387_V / 1_rad_per_s_sq}};

    static constexpr SwerveModuleConfig m_backRightConfig{
        126.650_deg,
        {
            3.26,
            0.0,
            0.0,
            kMaxModuleLinearAcceleration,
            kMaxModuleLinearJerk},
        {0.943,
         0.0,
         0.85,
         kMaxModuleAngularVelocity,
         kMaxModuleAngularAcceleration},
        {0.668_V,
         2.39_V / 1_mps,
         0.143_V / 1_mps_sq},
        {1.19_V,
         0.575_V / 1_rad_per_s,
         0.0496_V / 1_rad_per_s_sq}};
    
    // Trajectory Following
    frc::HolonomicDriveController m_trajectoryController{
        frc2::PIDController{2.0, 0.0, 0.0}, // X-error
        frc2::PIDController{2.0, 0.0, 0.0}, // Y-error
        frc::ProfiledPIDController<units::radian>{1.0, 0.0, 0.0,  // Rotation-error
            frc::TrapezoidProfile<units::radian>::Constraints{
                360_deg_per_s, 
                720_deg_per_s / 1_s
            }
        }
    };
};
