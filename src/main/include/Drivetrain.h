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

#include "ctre/Phoenix.h"
#include "adi/ADIS16470_IMU.h"

/**
 * Represents a differential drive style drivetrain.
 */
class Drivetrain
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
        m_driveL0.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, pidIdx);
        m_driveL0.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, timeoutMs);

        m_driveR0.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, pidIdx);
        m_driveR0.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, timeoutMs);

        m_gyro.Reset();

#ifdef __FRC_ROBORIO__
        m_imu.Reset();
#endif

        // Set the distance per pulse for the drive encoders. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        auto dpp = 2 * wpi::math::pi * kWheelRadius / kEncoderResolution;
        m_leftEncoder.SetDistancePerPulse(dpp.value());
        m_rightEncoder.SetDistancePerPulse(dpp.value());

        m_leftEncoder.Reset();
        m_rightEncoder.Reset();

        m_rightGroup.SetInverted(true);

        frc::SmartDashboard::PutData("Field", &m_fieldSim);
    }

    static constexpr units::meters_per_second_t kMaxSpeed =
        3.0_mps; // 3 meters per second
    static constexpr units::radians_per_second_t kMaxAngularSpeed{
        wpi::math::pi}; // 1/2 rotation per second

    void SetSpeeds(const frc::DifferentialDriveWheelSpeeds &speeds);
    void Drive(units::meters_per_second_t xSpeed,
               units::radians_per_second_t rot);
    void UpdateOdometry();
    void ResetOdometry(const frc::Pose2d &pose);

    frc::Pose2d GetPose() const { return m_odometry.GetPose(); }

    void SimulationPeriodic();
    void Periodic();

private:
    static constexpr units::meter_t kTrackWidth = 0.600_m;
    static constexpr units::meter_t kWheelRadius = 0.0762_m;
    static constexpr int kEncoderResolution = 2048;

    bool m_isSimulation = false;

    // TODO: Falcon 500  (DONE!)
    frc::PWMVictorSPX m_leftLeader{1};
    frc::PWMVictorSPX m_leftFollower{2};
    frc::PWMVictorSPX m_rightLeader{3};
    frc::PWMVictorSPX m_rightFollower{4};

    WPI_TalonFX m_driveL0{1};
    WPI_TalonFX m_driveL1{2};
    WPI_TalonFX m_driveL2{3};
    WPI_TalonFX m_driveR0{4};
    WPI_TalonFX m_driveR1{5};
    WPI_TalonFX m_driveR2{6};

    frc::SpeedControllerGroup m_leftGroup{
        m_leftLeader, 
        m_leftFollower,
        m_driveL0,
        m_driveL1,
        m_driveL2
    };

    frc::SpeedControllerGroup m_rightGroup{
        m_rightLeader, 
        m_rightFollower,
        m_driveR0,
        m_driveR1,
        m_driveR2
    };

    // TODO: Read from Falcon 500 (DONE!)
    frc::Encoder m_leftEncoder{0, 1};
    frc::Encoder m_rightEncoder{2, 3};

    // TODO-2: Tune on real robot
    frc2::PIDController m_leftPIDController{8.5, 0.0, 0.0};
    frc2::PIDController m_rightPIDController{8.5, 0.0, 0.0};

    // TODO: ADI Gyro
    frc::AnalogGyro m_gyro{0};

#ifdef __FRC_ROBORIO__
    frc::ADIS16470_IMU m_imu{
        frc::ADIS16470_IMU::IMUAxis::kZ, 
        frc::SPI::Port::kMXP, 
        frc::ADIS16470CalibrationTime::_512ms
    };
#endif

    frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
    frc::DifferentialDriveOdometry m_odometry{m_gyro.GetRotation2d()};

    // Gains are for example purposes only - must be determined for your own robot!
    frc::SimpleMotorFeedforward<units::meters> m_feedforward{1_V, 3_V / 1_mps};

    // Simulation classes help us simulate our robot
    frc::sim::AnalogGyroSim m_gyroSim{m_gyro};
    frc::sim::EncoderSim m_leftEncoderSim{m_leftEncoder};
    frc::sim::EncoderSim m_rightEncoderSim{m_rightEncoder};
    frc::Field2d m_fieldSim;

    // TODO-2: Tune on real robot
    decltype(1_V / 1_mps) kVlinear = 1.98_V / 1_mps;
    decltype(1_V / 1_mps_sq) kAlinear = 0.2_V / 1_mps_sq;
    decltype(1_V / 1_rad_per_s) kVangular = 1.5_V / 1_rad_per_s;
    decltype(1_V / 1_rad_per_s_sq) kAangular = 0.3_V / 1_rad_per_s_sq;

    frc::LinearSystem<2, 2, 2> m_drivetrainSystem =
        frc::LinearSystemId::IdentifyDrivetrainSystem(
            kVlinear, kAlinear,
            kVangular, kAangular);

    // TODO-2: Check real robot values
    frc::sim::DifferentialDrivetrainSim m_drivetrainSimulator{
        m_drivetrainSystem, 
        kTrackWidth, 
        frc::DCMotor::Falcon500(2), 
        10.24, 
        kWheelRadius
    };
};
