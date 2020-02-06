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
    return motorLeft1.GetSelectedSensorPosition(0) * kScaleFactor;
}

double Drivebase::GetEncoderPositionRight()
{
    return motorRight1.GetSelectedSensorPosition(0) * kScaleFactor;
}

double Drivebase::GetEncoderPosition()
{
    return (GetEncoderPositionLeft() + GetEncoderPositionRight()) / 2.0;
}

// Gyro
void Drivebase::ResetGyro()
{
    navx.ZeroYaw();
    forwardHeading = 0;
}

double Drivebase::GetGyroHeading()
{
    double yaw = -navx.GetYaw();
    return yaw;
}

void Drivebase::DriveForward(double distance, double maxOutput)
{

    double averageEncCnt = GetEncoderPosition();
    double error = distance - averageEncCnt;
    if (error < 24)
    {
        sumError_forward += error;
    }
    else
    {
        sumError_forward = 0;
    }
    double deltaError = (error - prevError_forward);
    prevError_forward = error;

    double driveCommandForward = error * KP_FORWARD + sumError_forward * KI_FORWARD + KD_FORWARD * deltaError;

    double gyroAngle = GetGyroHeading(); // -180 ~ 180

    double errorRot = forwardHeading - gyroAngle;

    if (errorRot > 180.0)
        errorRot -= 360.0;
    if (errorRot < -180.0)
        errorRot += 360.0;

    if (errorRot < 10)
    {
        sumError_rotation += errorRot / 0.02;
    }
    else
    {
        sumError_rotation = 0;
    }
    double deltaErrorRot = errorRot - prevError_rot;
    prevError_rot = error;

    double driveCommandRotation = errorRot * KP_ROTATION + KI_ROTATION * sumError_rotation + KD_ROTATION * deltaErrorRot;

    if (abs(driveCommandRotation) > 0.5)
    {
        if (driveCommandRotation > 0)
        {
            driveCommandRotation = 0.5;
        }
        else
        {
            driveCommandRotation = -0.5;
        }
    }

    if (driveCommandForward > maxOutput)
    {
        driveCommandForward = maxOutput;
    }
    if (driveCommandForward < -maxOutput)
    {
        driveCommandForward = -maxOutput;
    }

    Arcade(driveCommandForward, driveCommandRotation * maxOutput);
}

void Drivebase::TurnAbs(double heading, double maxoutput)
{
    forwardHeading = heading;
    double errorRot = forwardHeading - GetGyroHeading();
    if (errorRot > 180.0)
        errorRot -= 360.0;
    if (errorRot < -180.0)
        errorRot += 360.0;
    double deltaErrorRot = errorRot - prevError_rot;
    prevError_rot = errorRot;

    double driveCommandRotation = (errorRot * KP_ROTATION) + (KD_ROTATION * deltaErrorRot);

    if (driveCommandRotation > maxoutput)
    {
        driveCommandRotation = maxoutput;
    }
    if (driveCommandRotation < -maxoutput)
    {
        driveCommandRotation = -maxoutput;
    }

    Arcade(0, driveCommandRotation);
}


bool Drivebase::TurnRel(double degrees, double tolerance)
{
    target = GetGyroHeading() + degrees;
    double error = target - navx.GetYaw();
    //std::cout << error << std::endl;

    if (std::abs(error) < tolerance)
    {
        return true;
        Arcade(0.0, 0.0);
    }

    if (std::abs(error) < 8)
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

// SmartDash updater
void Drivebase::UpdateSmartdash()
{
    SmartDashboard::PutNumber("Drive L1", motorLeft1.Get());
    SmartDashboard::PutNumber("Drive L2", motorLeft2.Get());
    //SmartDashboard::PutNumber("Drive L3", motorLeft3.Get());
    SmartDashboard::PutNumber("Drive R1", motorRight1.Get());
    SmartDashboard::PutNumber("Drive R2", motorRight2.Get());
    //SmartDashboard::PutNumber("Drive R3", motorRight3.Get());

    SmartDashboard::PutBoolean("DriveShifter", solenoidShifter.Get());

    SmartDashboard::PutNumber("DriveEncL", GetEncoderPositionLeft());
    SmartDashboard::PutNumber("DriveEncR", GetEncoderPositionRight());

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

}
