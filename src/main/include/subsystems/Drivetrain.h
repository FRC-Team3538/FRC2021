// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AnalogGyro.h>
#include <frc/Encoder.h>
#include <frc/PWMVictorSPX.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/simulation/AnalogGyroSim.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <wpi/math>
#include <math.h>

#include "ctre/Phoenix.h"
#include "adi/ADIS16470_IMU.h"
#include "lib/Loggable.hpp"

/**
 * Represents a differential drive style drivetrain.
 */
class Drivetrain: public rj::Loggable
{
public:
    Drivetrain(bool isSimulation);

    void SetSpeeds(const frc::DifferentialDriveWheelSpeeds &speeds);
    void Drive(units::meters_per_second_t xSpeed,
               units::radians_per_second_t rot);
    void UpdateOdometry();
    void ResetOdometry(const frc::Pose2d &pose);

    frc::Pose2d GetPose() const { return m_odometry.GetPose(); }

    void SimulationPeriodic();
    void Periodic();

    void Log(UDPLogger &logger)
    {
        logger.LogExternalDevice(m_driveL0);
        logger.LogExternalDevice(m_driveL1);
        logger.LogExternalDevice(m_driveL2);
        logger.LogExternalDevice(m_driveR0);
        logger.LogExternalDevice(m_driveR1);
        logger.LogExternalDevice(m_driveR2);

#ifdef __FRC_ROBORIO__
        logger.LogExternalDevice(m_imu);
#endif
    }

private:
    /***************************************************************************/
    // CrossFire Characterization Values

    static constexpr units::meter_t kTrackWidth = 24.19872_in;
    static constexpr units::meter_t kWheelRadius = 3_in;
    static constexpr double kGearRatio = 7.09;
    static constexpr int kEncoderResolution = 2048;
    static constexpr int kMotorCount = 3;
    
    //static constexpr auto kScaleFactor = (2 * wpi::math::pi * kWheelRadius) / (kGearRatio * kEncoderResolution);
    static constexpr auto kScaleFactor = 240_in / 258824.5;

    decltype(1_V) kStatic{0.668};
    decltype(1_V / 1_fps) kVlinear{0.686};
    decltype(1_V / 1_fps_sq) kAlinear{0.124};
    decltype(1_V / 1_rad_per_s) kVangular{0.717};
    decltype(1_V / 1_rad_per_s_sq) kAangular{0.092};

    // Velocity Control PID
    frc2::PIDController m_leftPIDController{0.8382, 0.0, 0.0};
    frc2::PIDController m_rightPIDController{0.8382, 0.0, 0.0};

public:
    // Teleop Values
    /// TODO(Dereck): Measure these too
    static constexpr units::feet_per_second_t kMaxSpeed{10.0};
    static constexpr units::degrees_per_second_t kMaxAngularSpeed{180.0};

    /***************************************************************************/

private:
    bool m_isSimulation = false;

    // Simulation motor controllers
    frc::PWMVictorSPX m_leftLeader{1};
    frc::PWMVictorSPX m_leftFollower{2};
    frc::PWMVictorSPX m_rightLeader{3};
    frc::PWMVictorSPX m_rightFollower{4};

    // Real motor Controllers
    WPI_TalonFX m_driveL0{0};
    WPI_TalonFX m_driveL1{1};
    WPI_TalonFX m_driveL2{2};
    WPI_TalonFX m_driveR0{3};
    WPI_TalonFX m_driveR1{4};
    WPI_TalonFX m_driveR2{5};

    // Controller Groups
    frc::SpeedControllerGroup m_leftGroup{
        m_leftLeader,
        m_leftFollower,
        m_driveL0,
        m_driveL1,
        m_driveL2};

    frc::SpeedControllerGroup m_rightGroup{
        m_rightLeader,
        m_rightFollower,
        m_driveR0,
        m_driveR1,
        m_driveR2};

#ifdef __FRC_ROBORIO__
    frc::ADIS16470_IMU m_imu{
        frc::ADIS16470_IMU::IMUAxis::kX, //kZ
        frc::SPI::Port::kOnboardCS0,
        frc::ADIS16470CalibrationTime::_4s};
#else
    // Simulated Gyro (Linux Desktop Support)
    frc::AnalogGyro m_imu{0};
#endif

    //
    // Dynamics
    //
    frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
    frc::DifferentialDriveOdometry m_odometry{m_imu.GetRotation2d()};
    frc::SimpleMotorFeedforward<units::meters> m_feedforward{kStatic, kVlinear, kAlinear};

    //
    // Simulation
    //
    frc::LinearSystem<2, 2, 2> m_drivetrainSystem =
        frc::LinearSystemId::IdentifyDrivetrainSystem(
            kVlinear, kAlinear,
            kVangular, kAangular);

    frc::sim::DifferentialDrivetrainSim m_drivetrainSimulator{
        m_drivetrainSystem,
        kTrackWidth,
        frc::DCMotor::Falcon500(kMotorCount),
        kGearRatio,
        kWheelRadius};

    frc::Field2d m_fieldSim;
};
