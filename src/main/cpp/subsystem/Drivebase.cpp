#include "subsystem/Drivebase.hpp"
#include "frc/Timer.h"

#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>

Drivebase::Drivebase()
{
    // CTRE CAN
    motorLeft1.ConfigFactoryDefault();
    motorLeft2.ConfigFactoryDefault();
    motorRight1.ConfigFactoryDefault();
    motorRight2.ConfigFactoryDefault();

    // Invert one side of the drive
    motorLeft1.SetInverted(false);
    motorLeft2.SetInverted(false);

    motorRight1.SetInverted(true);
    motorRight2.SetInverted(true);

    // Encoder Feedback
    motorLeft1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0);
    motorLeft2.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0);
    motorLeft1.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);
    motorLeft2.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);

    motorRight1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0);
    motorRight2.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0);
    motorRight1.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);
    motorRight2.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);

    motorLeft1.SetSensorPhase(false);
    motorLeft2.SetSensorPhase(false);
    motorRight1.SetSensorPhase(false);
    motorRight2.SetSensorPhase(false);

    // set default shifter state
    solenoidShifter.Set(false);

    // Set Default Overrride State
    SensorOverride(false);

    // Smart Dash Stuff
    motorLeft1.Config_kF(slots::Forward, 0.0);
    motorLeft1.Config_kP(slots::Forward, 0.1);
    motorLeft1.Config_kI(slots::Forward, 0.0);
    motorLeft1.Config_kD(slots::Forward, 0.04);

    motorLeft2.Config_kF(slots::Forward, 0.0);
    motorLeft2.Config_kP(slots::Forward, 0.1);
    motorLeft2.Config_kI(slots::Forward, 0.0);
    motorLeft2.Config_kD(slots::Forward, 0.04);

    motorRight1.Config_kF(slots::Forward, 0.0);
    motorRight1.Config_kP(slots::Forward, 0.1);
    motorRight1.Config_kI(slots::Forward, 0.0);
    motorRight1.Config_kD(slots::Forward, 0.04);

    motorRight2.Config_kF(slots::Forward, 0.0);
    motorRight2.Config_kP(slots::Forward, 0.1);
    motorRight2.Config_kI(slots::Forward, 0.0);
    motorRight2.Config_kD(slots::Forward, 0.04);

    motorLeft1.ConfigNominalOutputForward(0);
    motorLeft1.ConfigNominalOutputReverse(0);
    motorLeft1.ConfigPeakOutputForward(1);
    motorLeft1.ConfigPeakOutputReverse(-1);

    motorLeft2.ConfigNominalOutputForward(0);
    motorLeft2.ConfigNominalOutputReverse(0);
    motorLeft2.ConfigPeakOutputForward(1);
    motorLeft2.ConfigPeakOutputReverse(-1);

    motorRight1.ConfigNominalOutputForward(0);
    motorRight1.ConfigNominalOutputReverse(0);
    motorRight1.ConfigPeakOutputForward(1);
    motorRight1.ConfigPeakOutputReverse(-1);

    motorRight2.ConfigNominalOutputForward(0);
    motorRight2.ConfigNominalOutputReverse(0);
    motorRight2.ConfigPeakOutputForward(1);
    motorRight2.ConfigPeakOutputReverse(-1);

    chooseDriveLimit.SetDefaultOption(sLimited, sLimited);
    chooseDriveLimit.AddOption(sUnlimited, sUnlimited);
}

// Arcade Drive
void Drivebase::Arcade(double forward, double turn)
{
    // Constrain input to +/- 1.0
    if (std::abs(forward) > 1.0)
    {
        forward /= std::abs(forward);
    }
    if (std::abs(turn) > 1.0)
    {
        turn /= std::abs(turn);
    }

    // CAN
    motorLeft1.Set(forward - turn);
    motorLeft2.Set(forward - turn);
    motorRight1.Set(forward + turn);
    motorRight2.Set(forward + turn);
}

// Stop! Hammer Time....
void Drivebase::Stop()
{
    motorLeft1.Set(0.0);
    motorLeft2.Set(0.0);
    motorRight1.Set(0.0);
    motorRight2.Set(0.0);
    // CAN
    motorLeft1.StopMotor();
    motorLeft2.StopMotor();
    motorRight1.StopMotor();
    motorRight2.StopMotor();
}

// Shift to High Gear
void Drivebase::SetHighGear()
{
    solenoidShifter.Set(true);
}

// Shift to Low Gear
void Drivebase::SetLowGear()
{
    solenoidShifter.Set(false);
}

//Coast Mode
void Drivebase::SetCoast()
{
    motorLeft1.SetNeutralMode(NeutralMode::Coast);
    motorLeft2.SetNeutralMode(NeutralMode::Coast);
    motorRight1.SetNeutralMode(NeutralMode::Coast);
    motorRight2.SetNeutralMode(NeutralMode::Coast);
}

//Brake Mode
void Drivebase::SetBrake()
{
    motorLeft1.SetNeutralMode(NeutralMode::Brake);
    motorLeft2.SetNeutralMode(NeutralMode::Brake);
    motorRight1.SetNeutralMode(NeutralMode::Brake);
    motorRight2.SetNeutralMode(NeutralMode::Brake);
}

// Reset Encoders
void Drivebase::ResetEncoders()
{
    motorLeft1.SetSelectedSensorPosition(0);
    motorLeft2.SetSelectedSensorPosition(0);
    motorRight1.SetSelectedSensorPosition(0);
    motorRight2.SetSelectedSensorPosition(0);
}

double Drivebase::GetEncoderPositionLeft()
{
    return motorLeft1.GetSelectedSensorPosition(0);// * kScaleFactor;
}

double Drivebase::GetEncoderPositionRight()
{
    return motorRight1.GetSelectedSensorPosition(0);// * kScaleFactor;
}

double Drivebase::GetEncoderPosition()
{
    return (GetEncoderPositionLeft() + GetEncoderPositionRight()) / 2.0;
}

// Gyro
void Drivebase::ResetGyro()
{
    m_imu.Reset();
    forwardHeading = 0;
}

double Drivebase::GetGyroHeading()
{
    double yaw = m_imu.GetAngle();
    return yaw;
}

bool Drivebase::VisionAim(double forward, double degrees, double tolerance)
{
    forwardHeading = GetGyroHeading() + degrees;

    double error = degrees;

    if (std::abs(error) < tolerance)
    {
        Arcade(forward, 0.0);
        return true;
    }

    if (std::abs(error) < 6.0)
    {
        iAcc += error / 0.02;
    }
    else
    {
        iAcc = 0;
    }

    double dError = (error - prevError_rel) / 0.02; // [Inches/second]
    prevError_rel = error;
    if (std::abs(forward) > 0.1)
    {
        Arcade(forward, ((error * KP_FORWARDGYRO) + (iAcc * KI_FORWARDGYRO) + (dError * KD_FORWARDGYRO)));
    }
    else
    {
        Arcade(forward, ((error * KP_ROTATIONV) + (iAcc * KI_ROTATIONV) + (dError * KD_ROTATIONV)));
    }

    return false;
}

void Drivebase::DriveForward(double distance, double maxOutput)
{

    double averageEncCnt = GetEncoderPositionLeft();
    double error = distance - averageEncCnt;
    if (abs(error) > 0.25 && abs(error) < 12.0)
    {
        sumError_forward += error;
    }
    else
    {
        sumError_forward = 0;
    }

    std::cout << error << " " << sumError_forward << std::endl;

    double deltaError = (error - prevError_forward);
    prevError_forward = error;

    double driveCommandForward = error * KP_FORWARD + sumError_forward * KI_FORWARD + KD_FORWARD * deltaError;

    double gyroAngle = GetGyroHeading(); // -180 ~ 180

    double errorRot = forwardHeading - gyroAngle;

    if (errorRot > 180.0)
        errorRot -= 360.0;
    if (errorRot < -180.0)
        errorRot += 360.0;

    if (abs(errorRot) > 0.5 && abs(errorRot) < 10)
    {
        sumError_rotation += errorRot;
    }
    else
    {
        sumError_rotation = 0;
    }
    double deltaErrorRot = errorRot - prevError_rot;
    prevError_rot = error;

    double driveCommandRotation = errorRot * KP_FORWARDGYRO + KI_FORWARDGYRO * sumError_rotation + KD_FORWARDGYRO * deltaErrorRot;

    //Drive Limits
    if (driveCommandRotation > maxOutput)
    {
        driveCommandRotation = maxOutput;
    }
    if (driveCommandRotation < -maxOutput)
    {
        driveCommandRotation = -maxOutput;
    }

    if (driveCommandForward > maxOutput)
    {
        driveCommandForward = maxOutput;
    }
    if (driveCommandForward < -maxOutput)
    {
        driveCommandForward = -maxOutput;
    }

    Arcade(driveCommandForward, driveCommandRotation);
}

void Drivebase::TurnAbs(double heading, double maxoutput)
{
    forwardHeading = heading;
    double errorRot = forwardHeading - GetGyroHeading();
    if (errorRot > 180.0)
        errorRot -= 360.0;
    if (errorRot < -180.0)
        errorRot += 360.0;

    if (abs(errorRot) > 0.5 && abs(errorRot) < 10)
    {
        sumError_rotation += errorRot;
    }
    else
    {
        sumError_rotation = 0;
    }

    double deltaErrorRot = errorRot - prevError_rot;
    prevError_rot = errorRot;

    double driveCommandRotation = errorRot * KP_ROTATION + KI_ROTATION * sumError_rotation + KD_ROTATION * deltaErrorRot;
    //double driveCommandRotation = (errorRot * KP_ROTATION) + (KD_ROTATION * deltaErrorRot);

    if (driveCommandRotation > maxoutput)
    {
        driveCommandRotation = maxoutput;
    }
    if (driveCommandRotation < -maxoutput)
    {
        driveCommandRotation = -maxoutput;
    }
    if ((abs(errorRot) > 0.4) && (driveCommandRotation < 0.085 && driveCommandRotation > 0.0))
    {
        driveCommandRotation = 0.09; //Minimum drive gain was 0.1
    }
    else if ((abs(errorRot) > 0.4) && (driveCommandRotation > -0.085 && driveCommandRotation < 0.0))
    {
        driveCommandRotation = -0.09;
    }
    Arcade(0, driveCommandRotation);
}

bool Drivebase::TurnRel(double degrees, double tolerance)
{
    // Save heading for other fuctions
    forwardHeading = GetGyroHeading() + degrees;

    double error = degrees;

    if (std::abs(error) < tolerance)
    {
        return true;
        Arcade(0.0, 0.0);
    }

    if (std::abs(error) < 6.0)
    {
        iAcc += error / 0.02;
    }
    else
    {
        iAcc = 0;
    }

    double dError = (error - prevError_rel) / 0.02; // [Inches/second]
    prevError_rel = error;

    Arcade(0.0, ((error * KP_ROTATION) + (iAcc * KI_ROTATION) + (dError * KD_ROTATION)));
    return false;
}

void Drivebase::GlobalReset()
{
    oneShotAngle = false;
    prevError_rotation = 0;
    prevError_forward = 0;
    sumError_forward = 0;
    prevError_rot = 0;
}

void Drivebase::SensorOverride(bool active)
{
    if (sensorOverride == active)
    {
        return;
    }
    sensorOverride = active;

    ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration lim;
    lim.currentLimit = 60;
    lim.triggerThresholdCurrent = 60;
    lim.triggerThresholdTime = 0.2;
    lim.enable = !sensorOverride;
    motorLeft1.ConfigSupplyCurrentLimit(lim);
    motorLeft2.ConfigSupplyCurrentLimit(lim);
    motorRight1.ConfigSupplyCurrentLimit(lim);
    motorRight2.ConfigSupplyCurrentLimit(lim);

    if (sensorOverride)
    {
        motorRight1.ConfigOpenloopRamp(0.0);
        motorRight2.ConfigOpenloopRamp(0.0);
        motorLeft1.ConfigOpenloopRamp(0.0);
        motorLeft2.ConfigOpenloopRamp(0.0);
    }
    else
    {
        motorRight1.ConfigOpenloopRamp(0.2);
        motorRight2.ConfigOpenloopRamp(0.2);
        motorLeft1.ConfigOpenloopRamp(0.2);
        motorLeft2.ConfigOpenloopRamp(0.2);
    }

    // TODO: Check if there are more settings that should be changed.
}

void Drivebase::UpdateOdometry()
{
    auto left = units::inch_t(GetEncoderPositionLeft());
    auto right = units::inch_t(GetEncoderPositionRight());
    m_odometry.Update(m_imu.GetRotation2d(),
                      units::meter_t(left),
                      units::meter_t(right));
}

void Drivebase::ResetOdometry(const frc::Pose2d &pose)
{
    ResetEncoders();

    m_odometry.ResetPosition(pose, pose.Rotation());
}

void Drivebase::MeasuredDrive(units::meters_per_second_t xSpeed,
                              units::radians_per_second_t rot)
{
    SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

void Drivebase::SetSpeeds(const frc::DifferentialDriveWheelSpeeds &speeds)
{
    auto leftFeedforward = m_feedforward.Calculate(speeds.left);
    auto rightFeedforward = m_feedforward.Calculate(speeds.right);

    double leftRate = 0.0;
    double rightRate = 0.0;

    leftRate = motorLeft1.GetSelectedSensorVelocity(0) * kScaleFactor * 10.0;
    rightRate = motorRight1.GetSelectedSensorVelocity(0) * kScaleFactor * 10.0;

    double leftOutput = m_leftPIDController.Calculate(
        leftRate,
        speeds.left.to<double>());

    double rightOutput = m_rightPIDController.Calculate(
        rightRate,
        speeds.right.to<double>());

    // motorLeft1.SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
    // motorLeft2.SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
    // motorRight1.SetVoltage(units::volt_t{rightOutput} + rightFeedforward);
    // motorRight2.SetVoltage(units::volt_t{rightOutput} + rightFeedforward);

    motorLeft1.SetVoltage(leftFeedforward);
    motorLeft2.SetVoltage(leftFeedforward);
    motorRight1.SetVoltage(rightFeedforward);
    motorRight2.SetVoltage(rightFeedforward);
}

// SmartDash updater
void Drivebase::UpdateSmartdash()
{

    SmartDashboard::PutNumber("Drive L1", motorLeft1.Get());
    //SmartDashboard::PutNumber("Drive L2", motorLeft2.Get());
    //SmartDashboard::PutNumber("Drive L3", motorLeft3.Get());
    SmartDashboard::PutNumber("Drive R1", motorRight1.Get());
    //SmartDashboard::PutNumber("Drive R2", motorRight2.Get());
    //SmartDashboard::PutNumber("Drive R3", motorRight3.Get());

    SmartDashboard::PutBoolean("DriveShifter", solenoidShifter.Get());

    SmartDashboard::PutNumber("DriveEncL", GetEncoderPositionLeft());
    SmartDashboard::PutNumber("DriveEncR", GetEncoderPositionRight());

    SmartDashboard::PutNumber("L1Tmp", motorLeft1.GetTemperature());
    SmartDashboard::PutNumber("L2Tmp", motorLeft2.GetTemperature());
    SmartDashboard::PutNumber("R1Tmp", motorRight1.GetTemperature());
    SmartDashboard::PutNumber("R2Tmp", motorRight2.GetTemperature());

    SmartDashboard::PutBoolean("DriveOverride", sensorOverride);

    SmartDashboard::PutNumber("Gyro Heading", GetGyroHeading());
    SmartDashboard::PutNumber("Heading Setpoint", forwardHeading);

    SmartDashboard::PutData("_DriveLimits", &chooseDriveLimit);
    SensorOverride(chooseDriveLimit.GetSelected() == sUnlimited);

    SmartDashboard::PutNumber("KP_FORWARD", KP_FORWARD);
    SmartDashboard::PutNumber("KI_FORWARD", KI_FORWARD);
    SmartDashboard::PutNumber("KD_FORWARD", KD_FORWARD);

    SmartDashboard::PutNumber("KP_ROTATION", KP_ROTATION);
    SmartDashboard::PutNumber("KI_ROTATION", KI_ROTATION);
    SmartDashboard::PutNumber("KD_ROTATION", KD_ROTATION);

    SmartDashboard::PutNumber("KP_ROTATIONV", KP_ROTATIONV);
    SmartDashboard::PutNumber("KI_ROTATIONV", KI_ROTATIONV);
    SmartDashboard::PutNumber("KD_ROTATIONV", KD_ROTATIONV);

    SmartDashboard::PutNumber("KP_FORWARDGYRO", KP_FORWARDGYRO);
    SmartDashboard::PutNumber("KI_FORWARDGYRO", KI_FORWARDGYRO);
    SmartDashboard::PutNumber("KD_FORWARDGYRO", KD_FORWARDGYRO);
}
