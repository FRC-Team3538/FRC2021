#include "subsystem/Drivebase.hpp"
#include "frc/Timer.h"

#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>

Drivebase::Drivebase()
{
    // PWM
   

    // CTRE CAN
    motorLeft1.ConfigFactoryDefault();
    motorLeft2.ConfigFactoryDefault();
    motorLeft3.ConfigFactoryDefault();
    motorRight1.ConfigFactoryDefault();
    motorRight2.ConfigFactoryDefault();
    motorRight3.ConfigFactoryDefault();

    // Invert one side of the drive
    motorLeft1.SetInverted(false);
    motorLeft2.SetInverted(false);
    motorLeft3.SetInverted(false);

    motorRight1.SetInverted(true);
    motorRight2.SetInverted(true);
    motorRight3.SetInverted(true);

    // Brake / Coast Mode
    motorLeft1.SetNeutralMode(NeutralMode::Brake);
    motorLeft2.SetNeutralMode(NeutralMode::Brake);
    motorLeft3.SetNeutralMode(NeutralMode::Brake);
    motorRight1.SetNeutralMode(NeutralMode::Brake);
    motorRight2.SetNeutralMode(NeutralMode::Brake);
    motorRight3.SetNeutralMode(NeutralMode::Brake);




    // set default shifter state
    solenoidShifter.Set(false);

    SensorOverride(false);

    // master > slaves
    motorLeft2.Follow(motorLeft1);
    motorLeft3.Follow(motorLeft1);

    motorRight2.Follow(motorRight1);
    motorRight3.Follow(motorRight1);

    // Encoder Feedback
    motorLeft1.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, PIDind::primary);
    motorLeft1.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);

    motorRight1.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, PIDind::primary);
    motorRight1.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);

    motorLeft1.SetSensorPhase(false);
    motorRight1.SetSensorPhase(false);

    motorLeft1.Config_kF(slots::Forward, 0.0);
    motorLeft1.Config_kP(slots::Forward, 0.1); 
    motorLeft1.Config_kI(slots::Forward, 0.0);
    motorLeft1.Config_kD(slots::Forward, 0.04);

    motorRight1.Config_kF(slots::Forward, 0.0);
    motorRight1.Config_kP(slots::Forward, 0.1);
    motorRight1.Config_kI(slots::Forward, 0.0);
    motorRight1.Config_kD(slots::Forward, 0.04);

    motorLeft1.ConfigNominalOutputForward(0);
    motorLeft1.ConfigNominalOutputReverse(0);
    motorLeft1.ConfigPeakOutputForward(1);
    motorLeft1.ConfigPeakOutputReverse(-1);

    motorRight1.ConfigNominalOutputForward(0);
    motorRight1.ConfigNominalOutputReverse(0);
    motorRight1.ConfigPeakOutputForward(1);
    motorRight1.ConfigPeakOutputReverse(-1);

    //Remote Sensor
    motorLeft1.ConfigRemoteFeedbackFilter(0, RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw, Remote0);
    motorRight1.ConfigRemoteFeedbackFilter(0, RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw, Remote0);

    motorLeft1.ConfigAuxPIDPolarity(true);
    motorRight1.ConfigAuxPIDPolarity(true);

    motorLeft1.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, PIDind::aux);
    motorRight1.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, PIDind::aux);

    motorLeft1.Config_kF(slots::Turning, 0.0);
    motorLeft1.Config_kP(slots::Turning, 0.25); //.25
    motorLeft1.Config_kI(slots::Turning, 0.00);
    motorLeft1.Config_kD(slots::Turning, 0.05); //.02

    motorRight1.Config_kF(slots::Turning, 0.0);
    motorRight1.Config_kP(slots::Turning, 0.25);
    motorRight1.Config_kI(slots::Turning, 0.000);
    motorRight1.Config_kD(slots::Turning, 0.05);

    motorLeft1.SelectProfileSlot(slots::Forward, PIDind::primary);
    motorLeft1.SelectProfileSlot(slots::Turning, PIDind::aux);

    motorRight1.SelectProfileSlot(slots::Forward, PIDind::primary);
    motorRight1.SelectProfileSlot(slots::Turning, PIDind::aux);
    motorLeft1.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 5);
    motorRight1.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 5);

    motorLeft1.ConfigSetParameter(ParamEnum::ePIDLoopPeriod, 1, 0x00, PIDind::aux);
    motorLeft1.ConfigSetParameter(ParamEnum::ePIDLoopPeriod, 1, 0x00, PIDind::primary);
    motorRight1.ConfigSetParameter(ParamEnum::ePIDLoopPeriod, 1, 0x00, PIDind::aux);
    motorRight1.ConfigSetParameter(ParamEnum::ePIDLoopPeriod, 1, 0x00, PIDind::primary);

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
    motorRight1.Set(forward + turn);

    // PWM
 
}

void Drivebase::Tank(double left, double right)
{
    // CAN
    motorLeft1.Set(left);
    motorRight1.Set(right);

    // PWM

}



// Stop!
void Drivebase::Stop()
{
    // CAN
    motorLeft1.StopMotor();
    motorRight1.StopMotor();
    
    // PWM
   
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

// Reset Encoders
void Drivebase::ResetEncoders()
{
    motorLeft1.SetSelectedSensorPosition(0);
    motorRight1.SetSelectedSensorPosition(0);
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
    double yaw = navx.GetYaw(); 
    return -yaw;
}

void Drivebase::DriveForward(double distance, double currentLimit)
{
    motorLeft1.ConfigNominalOutputForward(0);
    motorLeft1.ConfigNominalOutputReverse(0);
    motorLeft1.ConfigPeakOutputForward(currentLimit);  // TODO: Looks wrong, verify....
    motorLeft1.ConfigPeakOutputReverse(-currentLimit);

    motorRight1.ConfigNominalOutputForward(0);
    motorRight1.ConfigNominalOutputReverse(0);
    motorRight1.ConfigPeakOutputForward(currentLimit);
    motorRight1.ConfigPeakOutputReverse(-currentLimit);


    double averageEncCnt = GetEncoderPosition(); 
    double error = distance - averageEncCnt;
    if (error < 24)
    {
        sumError_forward += error / 0.02;
    }
    else
    {
        sumError_forward = 0;
    }
    double deltaError = (error - prevError_forward) / 0.02;
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

    Arcade(driveCommandForward, driveCommandRotation);
}

void Drivebase::Turn(double heading)
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

    Arcade(0, driveCommandRotation);
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
    motorRight1.ConfigSupplyCurrentLimit(lim);
        
    if(sensorOverride)
    {
        motorRight1.ConfigOpenloopRamp(0.0);
        motorLeft1.ConfigOpenloopRamp(0.0);
    } else {     
        motorRight1.ConfigOpenloopRamp(0.2);
        motorLeft1.ConfigOpenloopRamp(0.2);
    }

    // TODO: Check if there are more settings that should be changed.
}

// SmartDash updater
void Drivebase::UpdateSmartdash()
{
    SmartDashboard::PutNumber("DriveCmdL", motorLeft1.Get());
    SmartDashboard::PutNumber("DriveCmdR", motorRight1.Get());
    
    SmartDashboard::PutBoolean("DriveShifter", solenoidShifter.Get());

    SmartDashboard::PutNumber("DriveEncL", GetEncoderPositionLeft());
    SmartDashboard::PutNumber("DriveEncR", GetEncoderPositionRight());

    SmartDashboard::PutBoolean("DriveOverride", sensorOverride);

    SmartDashboard::PutNumber("Gyro Heading", GetGyroHeading());

    SmartDashboard::PutNumber("Heading Setpoint", forwardHeading);

    SmartDashboard::PutData("_DriveLimits", &chooseDriveLimit);
    SensorOverride(chooseDriveLimit.GetSelected() == sLimited);
}
