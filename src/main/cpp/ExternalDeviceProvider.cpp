#include "ExternalDeviceProvider.hpp"

#include "time.h"
#include <cmath>
#include <cstdlib>

flatbuffers::Offset<rj::CTREMotorStatusFrame>
GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, TalonFX &motor) {
  Faults faults;
  motor.GetFaults(faults);

  return rj::CreateCTREMotorStatusFrame(
      fbb, motor.GetFirmwareVersion(), motor.GetBaseID(), motor.GetDeviceID(),
      motor.GetOutputCurrent(), motor.GetBusVoltage(),
      motor.GetMotorOutputPercent(), motor.GetMotorOutputVoltage(),
      motor.GetTemperature(), motor.GetSelectedSensorPosition(),
      motor.GetSelectedSensorVelocity(), motor.GetClosedLoopError(),
      motor.GetIntegralAccumulator(), motor.GetErrorDerivative(),
      0, // TODO: only call this for valid modes. motor.GetClosedLoopTarget(),
      0, // TODO: only call this for valid modes.
         // motor.GetActiveTrajectoryPosition(),
      0, // TODO: only call this for valid modes.
         // motor.GetActiveTrajectoryVelocity(),
      0, // TODO: only call this for valid modes.
         // motor.GetActiveTrajectoryArbFeedFwd(),
      faults.ToBitfield(), motor.HasResetOccurred(), motor.GetLastError(),
      static_cast<int32_t>(motor.GetControlMode()), motor.GetStatorCurrent(),
      motor.GetSupplyCurrent(), motor.IsFwdLimitSwitchClosed(),
      motor.IsRevLimitSwitchClosed());
}

flatbuffers::Offset<rj::CTREMotorStatusFrame>
GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, TalonSRX &motor) {
  Faults faults;
  motor.GetFaults(faults);

  return rj::CreateCTREMotorStatusFrame(
      fbb, motor.GetFirmwareVersion(), motor.GetBaseID(), motor.GetDeviceID(),
      motor.GetOutputCurrent(), motor.GetBusVoltage(),
      motor.GetMotorOutputPercent(), motor.GetMotorOutputVoltage(),
      motor.GetTemperature(), motor.GetSelectedSensorPosition(),
      motor.GetSelectedSensorVelocity(), motor.GetClosedLoopError(),
      motor.GetIntegralAccumulator(), motor.GetErrorDerivative(),
      0, // TODO: only call this for valid modes. motor.GetClosedLoopTarget(),
      0, // TODO: only call this for valid modes.
         // motor.GetActiveTrajectoryPosition(),
      0, // TODO: only call this for valid modes.
         // motor.GetActiveTrajectoryVelocity(),
      0, // TODO: only call this for valid modes.
         // motor.GetActiveTrajectoryArbFeedFwd(),
      faults.ToBitfield(), motor.HasResetOccurred(), motor.GetLastError(),
      static_cast<int32_t>(motor.GetControlMode()), motor.GetStatorCurrent(),
      motor.GetSupplyCurrent(), motor.IsFwdLimitSwitchClosed(),
      motor.IsRevLimitSwitchClosed());
}

flatbuffers::Offset<rj::CTREMotorStatusFrame>
GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, VictorSPX &motor) {
  Faults faults;
  motor.GetFaults(faults);

  return rj::CreateCTREMotorStatusFrame(
      fbb, motor.GetFirmwareVersion(), motor.GetBaseID(), motor.GetDeviceID(),
      0.0, motor.GetBusVoltage(), motor.GetMotorOutputPercent(),
      motor.GetMotorOutputVoltage(), motor.GetTemperature(),
      motor.GetSelectedSensorPosition(), motor.GetSelectedSensorVelocity(),
      motor.GetClosedLoopError(), motor.GetIntegralAccumulator(),
      motor.GetErrorDerivative(),
      0, // TODO: only call this for valid modes. motor.GetClosedLoopTarget(),
      0, // TODO: only call this for valid modes.
         // motor.GetActiveTrajectoryPosition(),
      0, // TODO: only call this for valid modes.
         // motor.GetActiveTrajectoryVelocity(),
      0, // TODO: only call this for valid modes.
         // motor.GetActiveTrajectoryArbFeedFwd(),
      faults.ToBitfield(), motor.HasResetOccurred(), motor.GetLastError(),
      static_cast<int32_t>(motor.GetControlMode()), 0.0, 0.0, 0, 0);
}

rj::REVCANEncoder GetREVCANEncoder(rev::CANEncoder &encoder) {
  return rj::REVCANEncoder(
      encoder.GetPosition(),                 // position_
      encoder.GetVelocity(),                 // velocity_
      encoder.GetPositionConversionFactor(), // positionConversionFactor_
      encoder.GetVelocityConversionFactor(), // velocityConversaionFactor_
      encoder.GetAverageDepth(),             // averageDepth_
      encoder.GetMeasurementPeriod(),        // measurementPeriod_
      encoder.GetCountsPerRevolution(),      // countsPerRevolution_
      encoder.GetInverted(),                 // inverted_
      0 // encoder.GetLastError() // lastError_
  );
}

rj::REVCANAnalog GetREVCANAnalog(rev::CANAnalog &analog) {
  return rj::REVCANAnalog(
      analog.GetVoltage(),                  // voltage_
      analog.GetPosition(),                 // position_
      analog.GetVelocity(),                 // velocity_
      analog.GetPositionConversionFactor(), // positionConversionFactor_
      analog.GetVelocityConversionFactor(), // velocityConversaionFactor_
      analog.GetAverageDepth(),             // averageDepth_
      analog.GetMeasurementPeriod(),        // measurementPeriod_
      analog.GetInverted()                  // inverted_
  );
}

rj::REVPIDController GetREVPIDController(rev::CANPIDController &pid,
                                         int slotID) {
  return rj::REVPIDController(
      pid.GetP(slotID),                      // p_
      pid.GetI(slotID),                      // i_
      pid.GetD(slotID),                      // d_
      pid.GetDFilter(slotID),                // dFilter_
      pid.GetFF(slotID),                     // ff_
      pid.GetIZone(slotID),                  // iZone_
      pid.GetOutputMin(slotID),              // outputMin_
      pid.GetOutputMax(slotID),              // outputMax_
      pid.GetSmartMotionMaxVelocity(slotID), // smartMotionMaxVelocity_
      pid.GetSmartMotionMaxAccel(slotID),    // smartMotionMaxAccel_
      pid.GetSmartMotionMinOutputVelocity(
          slotID), // smartMotionMinOutputVelocity_
      pid.GetSmartMotionAllowedClosedLoopError(
          slotID),              // smartMotionAllowedClosedLoopError_
      pid.GetIMaxAccum(slotID), // iMaxAccum_
      pid.GetIAccum()           // iAccum_
  );
}

rj::REVCANDigitalInput GetRevCANDigitalInput(rev::CANDigitalInput &input) {
  return rj::REVCANDigitalInput(input.Get(), input.IsLimitSwitchEnabled());
}

flatbuffers::Offset<rj::REVMotorStatusFrame>
GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, rev::CANSparkMax &motor) {
  auto serialNo = motor.GetSerialNumber();
  auto encoder = motor.GetEncoder();
  auto analog = motor.GetAnalog();
  auto pid = motor.GetPIDController();
  auto fwdLimit = motor.GetForwardLimitSwitch(
      rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen);
  auto revLimit = motor.GetReverseLimitSwitch(
      rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen);

  auto revEncoder = GetREVCANEncoder(encoder);
  auto revAnalog = GetREVCANAnalog(analog);

  auto revPID0 = GetREVPIDController(pid, 0);
  auto revPID1 = GetREVPIDController(pid, 1);
  auto revPID2 = GetREVPIDController(pid, 2);
  auto revPID3 = GetREVPIDController(pid, 3);

  auto revFwdLimit = GetRevCANDigitalInput(fwdLimit);
  auto revRevLimit = GetRevCANDigitalInput(revLimit);

  return rj::CreateREVMotorStatusFrameDirect(
      fbb,
      motor.GetFirmwareVersion(),        // uint32_t firmwareVersion = 0,
      motor.GetFirmwareString().c_str(), // flatbuffers::Offset<flatbuffers::String>
                                         // firmwareString = 0,
      &serialNo,           // flatbuffers::Offset<flatbuffers::Vector<uint8_t>>
                           // serialNumber = 0,
      motor.GetDeviceId(), // int32_t deviceID = 0,
      &revEncoder,         // const rj::REVCANEncoder *encoder = 0,
      0,                   // const rj::REVCANEncoder *altEncoder = 0,
      &revAnalog,          // const rj::REVCANAnalog *analog = 0,
      &revPID0,            // const rj::REVPIDController *pid0 = 0,
      &revPID1,            // const rj::REVPIDController *pid1 = 0,
      &revPID2,            // const rj::REVPIDController *pid2 = 0,
      &revPID3,            // const rj::REVPIDController *pid3 = 0,
      &revFwdLimit,        // const rj::REVCANDigitalInput *fwdLimitSwitch = 0,
      &revRevLimit,        // const rj::REVCANDigitalInput *revLimitSwitch = 0,
      (bool)motor.GetIdleMode(),                    // bool idleMode = false,
      motor.GetVoltageCompensationNominalVoltage(), // double
                                                    // voltageCompensationNominalVoltage
                                                    // = 0.0,
      motor.GetOpenLoopRampRate(),   // double openLoopRampRate = 0.0,
      motor.GetClosedLoopRampRate(), // double closedLoopRampRate = 0.0,
      motor.GetBusVoltage(),         // double busVoltage = 0.0,
      motor.GetAppliedOutput(),      // double appliedOutput = 0.0,
      motor.GetOutputCurrent(),      // double outputCurrent = 0.0,
      motor.GetMotorTemperature(),   // double temperature = 0.0,
      motor.IsSoftLimitEnabled(
          rev::CANSparkMax::SoftLimitDirection::
              kForward), // bool softLimitForwardEnabled = false,
      motor.IsSoftLimitEnabled(
          rev::CANSparkMax::SoftLimitDirection::
              kReverse), // bool softLimitReverseEnabled = false,
      motor.GetSoftLimit(rev::CANSparkMax::SoftLimitDirection::
                             kForward), // double softLimitForwardValue = 0.0,
      motor.GetSoftLimit(rev::CANSparkMax::SoftLimitDirection::
                             kReverse), // double softLimitReverseValue = 0.0,
      (uint8_t)motor.GetLastError());   // uint8_t lastError = 0);
}

flatbuffers::Offset<rj::PDPStatusFrame>
GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb,
               frc::PowerDistributionPanel &pdp) {
  std::vector<double> currentMeasurements{
      pdp.GetCurrent(0),  pdp.GetCurrent(1),  pdp.GetCurrent(2),
      pdp.GetCurrent(3),  pdp.GetCurrent(4),  pdp.GetCurrent(5),
      pdp.GetCurrent(6),  pdp.GetCurrent(7),  pdp.GetCurrent(8),
      pdp.GetCurrent(9),  pdp.GetCurrent(10), pdp.GetCurrent(11),
      pdp.GetCurrent(12), pdp.GetCurrent(13), pdp.GetCurrent(14),
      pdp.GetCurrent(15)};

  return rj::CreatePDPStatusFrameDirect(
      fbb, pdp.GetModule(), pdp.GetVoltage(), pdp.GetTemperature(),
      &currentMeasurements, pdp.GetTotalCurrent(), pdp.GetTotalPower(),
      pdp.GetTotalEnergy());
}

flatbuffers::Offset<rj::PCMStatusFrame>
GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, frc::Compressor &pcm) {
  return rj::CreatePCMStatusFrame(
      fbb, pcm.GetModule(), pcm.Enabled(), pcm.GetPressureSwitchValue(),
      pcm.GetCompressorCurrent(), pcm.GetClosedLoopControl(),
      pcm.GetCompressorCurrentTooHighFault(), pcm.GetCompressorShortedFault(),
      pcm.GetCompressorNotConnectedFault());
}

rj::RawColor GetRawColor(rev::ColorSensorV3::RawColor &color) {
  return rj::RawColor(color.red, color.green, color.blue, color.ir);
}

flatbuffers::Offset<rj::REVColorSensorStatusFrame>
GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb,
               rev::ColorSensorV3 &colorSensor) {
  auto color = colorSensor.GetRawColor();
  auto revColor = GetRawColor(color);
  return rj::CreateREVColorSensorStatusFrame(
      fbb, &revColor, colorSensor.GetProximity(), colorSensor.HasReset());
}

rj::BoardYawAxis GetBoardYawAxis(AHRS::BoardYawAxis &yaw) {
  return rj::BoardYawAxis(yaw.board_axis, yaw.up);
}

flatbuffers::Offset<rj::NavXStatusFrame>
GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, AHRS &navx) {
  auto yaw = navx.GetBoardYawAxis();
  auto boardYaw = GetBoardYawAxis(yaw);

  return rj::CreateNavXStatusFrameDirect(
      fbb,
      navx.GetPitch(),                    // pitch:float;
      navx.GetRoll(),                     // roll:float;
      navx.GetYaw(),                      // yaw:float;
      navx.GetCompassHeading(),           // compassHeading:float;
      navx.IsCalibrating(),               // isCalibrating:bool;
      navx.IsConnected(),                 // isConnected:bool;
      navx.GetByteCount(),                // byteCount:double;
      navx.GetUpdateCount(),              // updateCount:double;
      navx.GetLastSensorTimestamp(),      // lastSensorTimestamp:uint64;
      navx.GetWorldLinearAccelX(),        // worldLinearAccelX:float;
      navx.GetWorldLinearAccelY(),        // worldLinearAccelY:float;
      navx.GetWorldLinearAccelZ(),        // worldLinearAccelZ:float;
      navx.IsMoving(),                    // isMoving:bool;
      navx.IsRotating(),                  // isRotating:bool;
      navx.GetBarometricPressure(),       // barometricPressure:float;
      navx.GetAltitude(),                 // altitude:float;
      navx.IsAltitudeValid(),             // isAltitudeValid:bool;
      navx.GetFusedHeading(),             // fusedHeading:float;
      navx.IsMagneticDisturbance(),       // isMagneticDisturbance:bool;
      navx.IsMagnetometerCalibrated(),    // isManetometerCalibrated:bool;
      navx.GetQuaternionW(),              // quaternionW:float;
      navx.GetQuaternionX(),              // quaternionX:float;
      navx.GetQuaternionY(),              // quaternionY:float;
      navx.GetQuaternionZ(),              // quaternionZ:float;
      navx.GetVelocityX(),                // velocityX:float;
      navx.GetVelocityY(),                // velocityY:float;
      navx.GetVelocityZ(),                // velocityZ:float;
      navx.GetDisplacementX(),            // displacementX:float;
      navx.GetDisplacementY(),            // displacementY:float;
      navx.GetDisplacementZ(),            // displacementZ:float;
      navx.GetAngle(),                    // angle:double;
      navx.GetRate(),                     // rate:double;
      navx.GetAngleAdjustment(),          // angleAdjustment:double;
      navx.GetRawGyroX(),                 // rawGyroX:float;
      navx.GetRawGyroY(),                 // rawGyroY:float;
      navx.GetRawGyroZ(),                 // rawGyroZ:float;
      navx.GetRawAccelX(),                // rawAccelX:float;
      navx.GetRawAccelY(),                // rawAccelY:float;
      navx.GetRawAccelZ(),                // rawAccelZ:float;
      navx.GetRawMagX(),                  // rawMagX:float;
      navx.GetRawMagY(),                  // rawMaxY:float;
      navx.GetRawMagZ(),                  // rawMagZ:float;
      navx.GetPressure(),                 // pressure:float;
      navx.GetTempC(),                    // tempC:float;
      &boardYaw,                          // boardYawAxis:BoardYawAxis;
      navx.GetFirmwareVersion().c_str(),  // firmwareVersion:string;
      navx.GetActualUpdateRate(),         // actualUpdateRate:int;
      navx.GetRequestedUpdateRate(),      // requestedUpdateRate:int;
      navx.IsBoardlevelYawResetEnabled(), // isBoardLevelYawResetEnabled:bool;
      navx.GetGyroFullScaleRangeDPS(),    // gyroFullScaleRangeDPS:int16;
      navx.GetAccelFullScaleRangeG()      // accelFullScaleRangeG:int16;
  );
}

flatbuffers::Offset<rj::ADIS16470StatusFrame>
GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, frc::ADIS16470_IMU &imu) {
  return rj::CreateADIS16470StatusFrame(
      fbb,
      imu.GetAngle(),
      imu.GetRate(),
      imu.GetGyroInstantX(),
      imu.GetGyroInstantY(),
      imu.GetGyroInstantZ(),
      imu.GetAccelInstantX(),
      imu.GetAccelInstantY(),
      imu.GetAccelInstantZ(),
      imu.GetXComplementaryAngle(),
      imu.GetYComplementaryAngle(),
      imu.GetXFilteredAccelAngle(),
      imu.GetYFilteredAccelAngle(),
      imu.GetYawAxis()
  );
}

flatbuffers::Offset<rj::WPIDigitalInput>
GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, frc::DigitalInput &input) {
  return rj::CreateWPIDigitalInput(
      fbb,
      input.GetChannel(),
      input.Get(),
      input.IsAnalogTrigger()
  );
}

flatbuffers::Offset<rj::WPIEncoder>
GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, frc::Encoder &encoder) {
  return rj::CreateWPIEncoder(
      fbb,
      encoder.Get(),
      encoder.GetPeriod(),
      encoder.GetStopped(),
      encoder.GetDirection(),
      encoder.GetRaw(),
      encoder.GetEncodingScale(),
      encoder.GetDistance(),
      encoder.GetRate(),
      encoder.GetDistancePerPulse(),
      encoder.GetSamplesToAverage(),
      encoder.PIDGet(),
      encoder.GetFPGAIndex()
  );
}

flatbuffers::Offset<rj::WPIDutyCycleEncoder>
GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, frc::DutyCycleEncoder &encoder) {
  return rj::CreateWPIDutyCycleEncoder(
      fbb,
      encoder.GetFrequency(),
      encoder.IsConnected(),
      (double) encoder.Get(),
      encoder.GetDistancePerRotation(),
      encoder.GetDistance(),
      encoder.GetFPGAIndex(),
      encoder.GetSourceChannel()
  );
}

void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder &fbb,
                              TalonFX &motor) {
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, motor);

  auto statusFrameHolder = rj::CreateStatusFrameHolder(
      fbb, unixTime, monotonicTime,
      rj::StatusFrame::StatusFrame_CTREMotorStatusFrame,
      statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder &fbb,
                              TalonSRX &motor) {
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, motor);

  auto statusFrameHolder = rj::CreateStatusFrameHolder(
      fbb, unixTime, monotonicTime,
      rj::StatusFrame::StatusFrame_CTREMotorStatusFrame,
      statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder &fbb,
                              VictorSPX &motor) {
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, motor);

  auto statusFrameHolder = rj::CreateStatusFrameHolder(
      fbb, unixTime, monotonicTime,
      rj::StatusFrame::StatusFrame_CTREMotorStatusFrame,
      statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder &fbb,
                              frc::PowerDistributionPanel &pdp) {
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, pdp);

  auto statusFrameHolder = rj::CreateStatusFrameHolder(
      fbb, unixTime, monotonicTime, rj::StatusFrame::StatusFrame_PDPStatusFrame,
      statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder &fbb,
                              frc::Compressor &pcm) {
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, pcm);

  auto statusFrameHolder = rj::CreateStatusFrameHolder(
      fbb, unixTime, monotonicTime, rj::StatusFrame::StatusFrame_PCMStatusFrame,
      statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder &fbb,
                              rev::CANSparkMax &motor) {
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, motor);

  auto statusFrameHolder = rj::CreateStatusFrameHolder(
      fbb, unixTime, monotonicTime,
      rj::StatusFrame::StatusFrame_REVMotorStatusFrame,
      statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder &fbb,
                              rev::ColorSensorV3 &colorSensor) {
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, colorSensor);

  auto statusFrameHolder = rj::CreateStatusFrameHolder(
      fbb, unixTime, monotonicTime,
      rj::StatusFrame::StatusFrame_REVColorSensorStatusFrame,
      statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder &fbb, AHRS &navx) {
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, navx);

  auto statusFrameHolder = rj::CreateStatusFrameHolder(
      fbb, unixTime, monotonicTime,
      rj::StatusFrame::StatusFrame_NavXStatusFrame, statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder &fbb, frc::ADIS16470_IMU &imu) {
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, imu);

  auto statusFrameHolder = rj::CreateStatusFrameHolder(
      fbb, unixTime, monotonicTime,
      rj::StatusFrame::StatusFrame_ADIS16470StatusFrame, statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder &fbb, frc::DigitalInput &input) {
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, input);

  auto statusFrameHolder = rj::CreateStatusFrameHolder(
      fbb, unixTime, monotonicTime,
      rj::StatusFrame::StatusFrame_WPIDigitalInput, statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder &fbb, frc::Encoder &encoder) {
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, encoder);

  auto statusFrameHolder = rj::CreateStatusFrameHolder(
      fbb, unixTime, monotonicTime,
      rj::StatusFrame::StatusFrame_WPIEncoder, statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder &fbb, frc::DutyCycleEncoder &encoder) {
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, encoder);

  auto statusFrameHolder = rj::CreateStatusFrameHolder(
      fbb, unixTime, monotonicTime,
      rj::StatusFrame::StatusFrame_WPIDutyCycleEncoder, statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void ExternalDeviceProvider::PopulateLogBuffer(UDPLogger &logger) {
  flatbuffers::DetachedBuffer buffer;

  // global
  fbb.Reset();
  BuildExternalDeviceFrame(fbb, pdp);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  fbb.Reset();
  BuildExternalDeviceFrame(fbb, pcm);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  // drivetrain
  fbb.Reset();
  BuildExternalDeviceFrame(fbb, driveLeft1);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  fbb.Reset();
  BuildExternalDeviceFrame(fbb, driveLeft2);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  fbb.Reset();
  BuildExternalDeviceFrame(fbb, driveRight1);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  fbb.Reset();
  BuildExternalDeviceFrame(fbb, driveRight2);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  // TODO: navx
  fbb.Reset();
  BuildExternalDeviceFrame(fbb, navx);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  // climber
  fbb.Reset();
  BuildExternalDeviceFrame(fbb, motorClimber0);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  fbb.Reset();
  BuildExternalDeviceFrame(fbb, motorClimber1);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  // color wheel
  fbb.Reset();
  BuildExternalDeviceFrame(fbb, motorColorWheel);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  fbb.Reset();
  BuildExternalDeviceFrame(fbb, m_colorSensor);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  // shooter
  fbb.Reset();
  BuildExternalDeviceFrame(fbb, flywheel);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  fbb.Reset();
  BuildExternalDeviceFrame(fbb, flywheelB);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  fbb.Reset();
  BuildExternalDeviceFrame(fbb, motorIntake);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  fbb.Reset();
  BuildExternalDeviceFrame(fbb, motorIndexer);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  fbb.Reset();
  BuildExternalDeviceFrame(fbb, motorIndexerB);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  fbb.Reset();
  BuildExternalDeviceFrame(fbb, motorIndexerC);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  fbb.Reset();
  BuildExternalDeviceFrame(fbb, sparkIndexerB);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  fbb.Reset();
  BuildExternalDeviceFrame(fbb, sparkIndexerC);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  fbb.Reset();
  BuildExternalDeviceFrame(fbb, motorFeeder);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  fbb.Reset();
  BuildExternalDeviceFrame(fbb, motorHood);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  fbb.Reset();
  BuildExternalDeviceFrame(fbb, hoodEncAbs);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  fbb.Reset();
  BuildExternalDeviceFrame(fbb, hoodZeroSw);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  fbb.Reset();
  BuildExternalDeviceFrame(fbb, hoodEncQuad);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());
}
