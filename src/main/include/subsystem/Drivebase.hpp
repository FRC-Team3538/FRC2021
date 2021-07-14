#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Solenoid.h>
#include <ctre/Phoenix.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/PWMTalonSRX.h>
#include <iostream>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include "adi/ADIS16470_IMU.h"
#include <frc/system/plant/LinearSystemId.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/simulation/AnalogGyroSim.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/Encoder.h>
// #include "rev/CANSparkMax.h"

using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;
using namespace frc;
// using namespace rev;

class Drivebase
{
private:
  // Hardware setup
  enum motors
  {
    L1 = 0,
    L2,
    L3,
    R1,
    R2,
    R3
  };

  // Talon
  WPI_TalonFX motorLeft1{motors::L1};
  WPI_TalonFX motorLeft2{motors::L2};

  WPI_TalonFX motorRight1{motors::R1};
  WPI_TalonFX motorRight2{motors::R2};

  Solenoid solenoidShifter{8};

  // Encoder Scale Factor (Inches)/(Pulse)
  const double kScaleFactor = 235.0 / 253119; //53.1875 / 52896;

  enum slots
  {
    Forward = 0,
    Turning,
    Slot2,
    Slot3
  };

  double prevError_rotation = 0;
  double prevError_forward = 0;
  double sumError_forward = 0;
  double prevError_rot = 0;
  double sumError_rotation = 0;
  double prevError_rel = 0.0;
  double iAcc = 0.0;

  bool oneShotAngle = false;

#define KP_ROTATIONV (SmartDashboard::GetNumber("KP_ROTATIONV", 0.018))
#define KI_ROTATIONV (SmartDashboard::GetNumber("KI_ROTATIONV", 0.000045)) //275
#define KD_ROTATIONV (SmartDashboard::GetNumber("KD_ROTATIONV", 0.001))    //15

#define KP_ROTATION (SmartDashboard::GetNumber("KP_ROTATION", 0.022))
#define KI_ROTATION (SmartDashboard::GetNumber("KI_ROTATION", 0.0002)) //275
#define KD_ROTATION (SmartDashboard::GetNumber("KD_ROTATION", 0.0015)) //15

#define KP_FORWARD (SmartDashboard::GetNumber("KP_FORWARD", 0.021)) //0.021
#define KI_FORWARD (SmartDashboard::GetNumber("KI_FORWARD", 0.00050))
#define KD_FORWARD (SmartDashboard::GetNumber("KD_FORWARD", 0.0))

#define KP_FORWARDGYRO (SmartDashboard::GetNumber("KP_FORWARDGYRO", 0.008)) //75
#define KI_FORWARDGYRO (SmartDashboard::GetNumber("KI_FORWARDGYRO", 0.0))
#define KD_FORWARDGYRO (SmartDashboard::GetNumber("KD_FORWARDGYRO", 0.0))

  SendableChooser<std::string> chooseDriveLimit;
  const std::string sLimited = "Normal";
  const std::string sUnlimited = "Override";

  static constexpr units::meter_t kTrackWidth = 24.19872_in;
  static constexpr units::meter_t kWheelRadius = 3_in;
  static constexpr double kGearRatio = 10.24;

  decltype(1_V) kStatic{0.689}; //.712
  decltype(1_V / 1_mps) kVlinear{2.35};  //2.3
  decltype(1_V / 1_mps_sq) kAlinear{0.216}; //.302
  decltype(1_V / 1_rad_per_s) kVangular{.185}; //2.3 1.85
  decltype(1_V / 1_rad_per_s_sq) kAangular{0.0152}; //.255 0.0152

  frc2::PIDController m_leftPIDController{0.8382, 0.0, 0.0};
  frc2::PIDController m_rightPIDController{0.8382, 0.0, 0.0};

  frc::ADIS16470_IMU m_imu{
      frc::ADIS16470_IMU::IMUAxis::kX,
      frc::SPI::Port::kOnboardCS0,
      frc::ADIS16470CalibrationTime::_4s};

  frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
  frc::DifferentialDriveOdometry m_odometry{m_imu.GetRotation2d()};
  frc::SimpleMotorFeedforward<units::meters> m_feedforward{kStatic, kVlinear, kAlinear};

public:
  // Default Constructor
  Drivebase();

  static constexpr units::feet_per_second_t kMaxSpeed{15.0};
  static constexpr units::degrees_per_second_t kMaxAngularSpeed{180.0};

  bool sensorOverride = false;
  double forwardHeading = 0;
  // Actions
  void Arcade(double forward, double rotate);

  void Stop();
  void SetHighGear();
  void SetLowGear();
  void SetCoast();
  void SetBrake();

  void SensorOverride(bool active);

  void ResetEncoders();
  double GetEncoderPositionLeft();
  double GetEncoderPositionRight();
  double GetEncoderPosition();

  void ResetGyro();
  double GetGyroHeading();
  double GetPitch();
  void GlobalReset();

  void UpdateSmartdash();

  bool VisionAim(double forward, double degrees, double tolerance);
  void DriveForward(double distance, double maxoutput = 1.0);
  void TurnAbs(double degrees, double maxoutput = 1.0);
  bool TurnRel(double degrees, double tolerance);
  void SetMaxSpeed();

  void UpdateOdometry();
  void ResetOdometry(const frc::Pose2d &pose);
  void MeasuredDrive(units::meters_per_second_t xSpeed,
                     units::radians_per_second_t rot);
  void SetSpeeds(const frc::DifferentialDriveWheelSpeeds &speeds);
  frc::Pose2d GetPose() const { return m_odometry.GetPose(); }

  frc::DifferentialDriveKinematics GetKinematics()
  {
    return m_kinematics;
  }

  frc::SimpleMotorFeedforward<units::meter> GetFeedForward()
  {
    return m_feedforward;
  }
};