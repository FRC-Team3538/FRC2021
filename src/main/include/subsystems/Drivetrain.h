// Copyright (c) FRC3538 - RoboJackets, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
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
    void Drive(frc::Trajectory::State trajectoryState, units::radian_t yaw = 0_rad);
    void UpdateOdometry();
    frc::Rotation2d GetYaw();
    units::radians_per_second_t GetYawRate();
    void ResetYaw();
    void Log(UDPLogger &logger);
    void SimPeriodic();
    void InitSendable(frc::SendableBuilder &builder) override;
    void ResetOdometry(const frc::Pose2d &pose);
    void ShowTrajectory(const frc::Trajectory &trajectory);

    static constexpr auto kMaxSpeed = 16_fps;
    // about 2.5 turns per second
    // calculated theoretically to 2.743821154 turns/sec
    static constexpr auto kMaxAngularSpeed = 16_rad_per_s;

private:
    // Configuration
    static constexpr auto m_dist = 15.75_in / 2;
    frc::Translation2d m_frontLeftLocation{+m_dist, +m_dist};
    frc::Translation2d m_frontRightLocation{+m_dist, -m_dist};
    frc::Translation2d m_backLeftLocation{-m_dist, +m_dist};
    frc::Translation2d m_backRightLocation{-m_dist, -m_dist};

    bool m_fieldRelative = true;

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
    units::radian_t m_theta = 0_rad;
#endif

    // Heading Lock
    bool m_YawLockActive = true;
    frc2::PIDController m_yawLockPID{5.0, 0.0, 0.1};

    // Odometry
    frc::SwerveDriveKinematics<4> m_kinematics{
        m_frontLeftLocation,
        m_frontRightLocation,
        m_backLeftLocation,
        m_backRightLocation};

    frc::SwerveDriveOdometry<4> m_odometry{
        m_kinematics,
        frc::Rotation2d(),
        frc::Pose2d()};

    frc::SwerveDrivePoseEstimator<4> m_poseEstimator{
        frc::Rotation2d(),
        frc::Pose2d(),
        m_kinematics,
        {0.5, 0.5, 0.05},
        {2.0},
        {0.0, 0.0, 0.0}
    };

    frc::ChassisSpeeds m_robotVelocity;

    frc::Field2d m_fieldDisplay;


    // Control
    frc::ChassisSpeeds m_command;

    static constexpr auto kMaxModuleLinearAcceleration = 80.0_mps_sq;
    static constexpr auto kMaxModuleLinearJerk = 200.0_mps_sq / 1_s;

    static constexpr auto kMaxModuleAngularVelocity = 18_rad_per_s;
    static constexpr auto kMaxModuleAngularAcceleration = 200_rad_per_s_sq;

    static constexpr SwerveModuleConfig m_frontLeftConfig{
        units::degree_t(-172.266),
        {2.42,
         0.0,
         0.0,
         kMaxModuleLinearAcceleration,
         kMaxModuleLinearJerk},
        {8.0, // 2.5179,
         0.0, // 0.0,
         0.5, // 0.15272,
         kMaxModuleAngularVelocity,
         kMaxModuleAngularAcceleration},
        {0.673_V,
         2.35_V / 1_mps,
         0.0937_V / 1_mps_sq},
        {0.49655_V,
         0.65857_V / 1_rad_per_s,
         0.042166_V / 1_rad_per_s_sq}};

    static constexpr SwerveModuleConfig m_frontRightConfig{
        units::degree_t(34.717),
        {2.42,
         0.0,
         0.0,
         kMaxModuleLinearAcceleration,
         kMaxModuleLinearJerk},
        {8.0, // 4.2946,
         0.0, // 0.0,
         0.5, // 0.050889,
         kMaxModuleAngularVelocity,
         kMaxModuleAngularAcceleration},
        {0.673_V,
         2.35_V / 1_mps,
         0.0937_V / 1_mps_sq},
        {0.58739_V,
         0.64399_V / 1_rad_per_s,
         0.018826_V / 1_rad_per_s_sq}};

    static constexpr SwerveModuleConfig m_backLeftConfig{
        units::degree_t(62.666),
        {2.42,
         0.0,
         0.0,
         kMaxModuleLinearAcceleration,
         kMaxModuleLinearJerk},
        {8.0, // 4.9251,
         0.0, // 0.0,
         0.5, // 0.048966,
         kMaxModuleAngularVelocity,
         kMaxModuleAngularAcceleration},
        {0.673_V,
         2.35_V / 1_mps,
         0.0937_V / 1_mps_sq},
        {1.0045_V,
         0.6584_V / 1_rad_per_s,
         0.015321_V / 1_rad_per_s_sq}};

    static constexpr SwerveModuleConfig m_backRightConfig{
        units::degree_t(25.137),
        {2.42,
         0.0,
         0.0,
         kMaxModuleLinearAcceleration,
         kMaxModuleLinearJerk},
        {8.0, // 3.5128,
         0.0, // 0.0,
         0.5, // 0.059802,
         kMaxModuleAngularVelocity,
         kMaxModuleAngularAcceleration},
        {0.673_V,
         2.35_V / 1_mps,
         0.0937_V / 1_mps_sq},
        {0.63069_V,
         0.6333_V / 1_rad_per_s,
         0.024316_V / 1_rad_per_s_sq}};

    // Trajectory Following
    frc::HolonomicDriveController m_trajectoryController{
        frc2::PIDController{2.0, 0.0, 0.0},                      // X-error
        frc2::PIDController{2.0, 0.0, 0.0},                      // Y-error
        frc::ProfiledPIDController<units::radian>{1.0, 0.0, 0.0, // Rotation-error
                                                  frc::TrapezoidProfile<units::radian>::Constraints{
                                                      360_deg_per_s,
                                                      720_deg_per_s / 1_s}}};
};
