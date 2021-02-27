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
#include <lib/Loggable.hpp>

/**
 * Represents a differential drive style drivetrain.
 */
class Drivetrain: public rj::Loggable
{
public:
    Drivetrain(bool isSimulation)
    {
        m_isSimulation = isSimulation;

        m_driveL0.ConfigFactoryDefault();
        m_driveL1.ConfigFactoryDefault();
        m_driveL2.ConfigFactoryDefault();
        m_driveR0.ConfigFactoryDefault();
        m_driveR1.ConfigFactoryDefault();
        m_driveR2.ConfigFactoryDefault();

        double pidIdx = 0;
        double timeoutMs = 18;
        m_driveL0.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, pidIdx);
        m_driveL0.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);

        m_driveR0.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, pidIdx);
        m_driveR0.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);

        m_gyro.Reset();

#ifdef __FRC_ROBORIO__
        m_imu.Reset();
#endif

        // Set the distance per pulse for the drive encoders. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        auto dpp = /*empiricalDist/196920.0;*/empiricalDist/258824.5;//((2 * wpi::math::pi * kWheelRadius) / kEncoderResolution) / 10.24;
        m_leftEncoder.SetDistancePerPulse(dpp.value());
        m_rightEncoder.SetDistancePerPulse(-dpp.value());

        m_leftEncoder.Reset();
        m_rightEncoder.Reset();

        m_rightGroup.SetInverted(true);
        m_leftGroup.SetInverted(false);

        m_driveL0.SetSelectedSensorPosition(0.0);
        m_driveR0.SetSelectedSensorPosition(0.0);

        m_driveL0.SetNeutralMode(NeutralMode::Brake);
        m_driveL1.SetNeutralMode(NeutralMode::Brake);
        m_driveL2.SetNeutralMode(NeutralMode::Brake);
        m_driveR0.SetNeutralMode(NeutralMode::Brake);
        m_driveR1.SetNeutralMode(NeutralMode::Brake);
        m_driveR2.SetNeutralMode(NeutralMode::Brake);

        frc::SmartDashboard::PutData("Field", &m_fieldSim);
    }

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
    static constexpr units::meter_t empiricalDist = 240_in;
    static constexpr double kGearRatio = 10.24;
    static constexpr int kEncoderResolution = 2048;
    static constexpr int kMotorCount = 2;

    decltype(1_V) kStatic{0.668};
    decltype(1_V / 1_fps) kVlinear{0.686};
    decltype(1_V / 1_fps_sq) kAlinear{0.124};
    decltype(1_V / 1_rad_per_s) kVangular{0.717};
    decltype(1_V / 1_rad_per_s_sq) kAangular{0.092};

    // Velocity Control PID (Is this really required ???)
    frc2::PIDController m_leftPIDController{0.8382, 0.0, 0.0}; //2.75
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

    // Simulated Encoders
    frc::Encoder m_leftEncoder{0, 1};
    frc::Encoder m_rightEncoder{2, 3};

    // Simulated Gyro
    frc::AnalogGyro m_gyro{0};

#ifdef __FRC_ROBORIO__
    frc::ADIS16470_IMU m_imu{
        frc::ADIS16470_IMU::IMUAxis::kX, //kZ
        frc::SPI::Port::kOnboardCS0,
        frc::ADIS16470CalibrationTime::_4s};
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

    frc::sim::AnalogGyroSim m_gyroSim{m_gyro};
    frc::sim::EncoderSim m_leftEncoderSim{m_leftEncoder};
    frc::sim::EncoderSim m_rightEncoderSim{m_rightEncoder};
    frc::Field2d m_fieldSim;
};
