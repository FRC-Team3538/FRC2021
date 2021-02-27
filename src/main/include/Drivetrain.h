/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
#include <ctre/Phoenix.h>
#include <adi/ADIS16470_IMU.h>
/**
 * Represents a differential drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain() {
#ifdef __FRC_ROBORIO__
    m_imu.Reset();
#else
    m_gyro.Reset();
#endif
    
    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //_leftEncoder.SetDistancePerPulse(kScaleFactor.value());
    //m_rightEncoder.SetDistancePerPulse(kScaleFactor.value());

    SetEncoderPositionLeft(0_m);
    SetEncoderPositionRight(0_m);

    m_rightGroup.SetInverted(true);

    frc::SmartDashboard::PutData("Field", &m_fieldSim);

    
  }

  static constexpr units::meters_per_second_t kMaxSpeed =
      3.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      wpi::math::pi};  // 1/2 rotation per second

  void SetSpeeds(const frc::DifferentialDriveWheelSpeeds& speeds);
  void Drive(units::meters_per_second_t xSpeed,
             units::radians_per_second_t rot);
  void UpdateOdometry();
  void ResetOdometry(const frc::Pose2d& pose);

  frc::Pose2d GetPose() const { return m_odometry.GetPose(); }

  void SimulationPeriodic();
  void Periodic();

  units::meter_t GetEncoderPositionLeft();
  units::meter_t GetEncoderPositionRight();
  units::meters_per_second_t GetEncoderVelocityLeft();
  units::meters_per_second_t GetEncoderVelocityRight();
  void SetEncoderPositionLeft(units::meter_t position);
  void SetEncoderPositionRight(units::meter_t position);

 private:
  
  static constexpr units::meter_t kTrackWidth = 0.381_m * 2;
  static constexpr units::meter_t kWheelRadius = 0.0508_m;  // meters
  static constexpr int kEncoderResolution = 4096;
  static constexpr units::meter_t kScaleFactor = 2.0 * wpi::math::pi * kWheelRadius / kEncoderResolution;

  WPI_TalonSRX m_leftLeader{1};
  WPI_TalonSRX m_leftFollower{2};
  WPI_TalonSRX m_rightLeader{3};
  WPI_TalonSRX m_rightFollower{4};

  WPI_TalonFX m_DriveL0{1};
  WPI_TalonFX m_DriveL1{2};
  WPI_TalonFX m_DriveL2{3};
  WPI_TalonFX m_DriveR0{4};
  WPI_TalonFX m_DriveR1{5};
  WPI_TalonFX m_DriveR2{6};
  
  frc::SpeedControllerGroup m_leftGroup{m_leftLeader, m_leftFollower, m_DriveL0, m_DriveL1, m_DriveL2};
  frc::SpeedControllerGroup m_rightGroup{m_rightLeader, m_rightFollower, m_DriveR0, m_DriveR1, m_DriveR2};


  frc::Encoder m_leftEncoder{0,1};
  frc::Encoder m_rightEncoder{2,3};


  frc2::PIDController m_leftPIDController{8.5, 0.0, 0.0};
  frc2::PIDController m_rightPIDController{8.5, 0.0, 0.0};

  frc::AnalogGyro m_gyro{0};

 #ifdef __FRC_ROBORIO__
    frc::ADIS16470_IMU m_imu{
        frc::ADIS16470_IMU::IMUAxis::kZ,
        frc::SPI::Port::kMXP,
        frc::ADIS16470CalibrationTime::_512ms};
#endif

  frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
  frc::DifferentialDriveOdometry m_odometry{m_gyro.GetRotation2d()};

  // Gains are for example purposes only - must be determined for your own
  // robot!
  frc::SimpleMotorFeedforward<units::meters> m_feedforward{1_V, 3_V / 1_mps};

  // Simulation classes help us simulate our robot
  frc::sim::AnalogGyroSim m_gyroSim{m_gyro};
 //frc::sim::EncoderSim m_leftEncoderSim{m_leftEncoder};
  //frc::sim::EncoderSim m_rightEncoderSim{m_rightEncoder};
  frc::Field2d m_fieldSim;
  frc::LinearSystem<2, 2, 2> m_drivetrainSystem =
      frc::LinearSystemId::IdentifyDrivetrainSystem(
          1.98_V / 1_mps, 0.2_V / 1_mps_sq, 1.5_V / 1_rad_per_s,
          0.3_V / 1_rad_per_s_sq);
  frc::sim::DifferentialDrivetrainSim m_drivetrainSimulator{
      m_drivetrainSystem, kTrackWidth, frc::DCMotor::Falcon500(2), 10.24, 3_in};
};
