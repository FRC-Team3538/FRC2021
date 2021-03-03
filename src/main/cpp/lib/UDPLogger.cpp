#include <iostream>
#include <assert.h>
#include <iostream>

#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/ip.h>
#include <sys/socket.h>
#endif

#include "frc/Timer.h"

#include "proto/StatusFrame_generated.h"
#include "lib/UDPLogger.hpp"

#if defined(_WIN32)

void UDPLogger::InitLogger()
{
}

void UDPLogger::CheckForNewClient()
{
}

void UDPLogger::FlushLogBuffer()
{
}

#else

// Helper to compare client socket addresses
inline bool operator==(const sockaddr_in &lhs, const sockaddr_in &rhs)
{
  return (lhs.sin_family == rhs.sin_family) && (lhs.sin_port == rhs.sin_port) && (lhs.sin_addr.s_addr == rhs.sin_addr.s_addr);
}

void UDPLogger::InitLogger()
{
  sockfd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, IPPROTO_UDP);

  if (sockfd < 0)
  {
    std::cout << "socket() failed! " << strerror(errno) << std::endl;
    return;
  }

  address.sin_family = AF_INET;
  address.sin_port = htons(m_udpPort);

  if (inet_aton("0.0.0.0", &address.sin_addr) == 0)
  {
    std::cout << "inet_aton() failed! " << strerror(errno) << std::endl;
    return;
  }

  if (bind(sockfd, (const struct sockaddr *)&address, sizeof(address)) != 0)
  {
    std::cout << "bind() failed! " << strerror(errno) << std::endl;
    return;
  }
}

int sendLog(int sockfd,
            const uint8_t *data,
            size_t size,
            const struct sockaddr_in &address)
{
  if (sockfd < 0)
  {
    std::cout << "RJ: Dirty sock! " << std::endl;
    return 0;
  }

  std::cout << "RJ: SENDING " << size << " bytes" << std::endl;
  return sendto(sockfd, data, size, 0, (const struct sockaddr *)&address, sizeof(address));
}

// Need to lock at this level so we don't cause iterator invalidation on `m_clients`
void UDPLogger::FlushLogBuffer()
{
  std::scoped_lock lock{mut};

  for (struct sockaddr_in client : m_clients)
  {
    if (sendLog(sockfd, buf, bufsize, client) < 0)
    {
      std::cout << "RJ: SendLog failed! " << strerror(errno) << std::endl;
    }
  }
  bufsize = 0;
}

void UDPLogger::CheckForNewClient()
{
  constexpr size_t bufferSz = 300;
  struct sockaddr_in client;
  socklen_t client_len = sizeof(struct sockaddr_in);
  char buf[bufferSz];
  ssize_t res_len = recvfrom(sockfd,
                             (void *)buf,
                             bufferSz - 1,
                             0,
                             (struct sockaddr *)&client,
                             &client_len);

  // No Requests = bail
  if(res_len < 0) return;

  // Parse Command
  buf[res_len] = '\n';
  auto sBuf = std::string(buf);

  if (sBuf.find("Hi") == 0)
  {
    // Connect
    std::cout << "RJ: Log Client Connected!" << std::endl;

    // Send connection info to client
    fbb.Reset();
    auto greeting = rj::CreateInitializeStatusFrameDirect(fbb, title.c_str());
    auto wrapper =
        rj::CreateStatusFrameHolder(fbb,
                                    frc::GetTime(),
                                    frc::Timer::GetFPGATimestamp(),
                                    rj::StatusFrame_InitializeStatusFrame,
                                    greeting.Union());
    fbb.FinishSizePrefixed(wrapper);
    auto buffer = fbb.Release();

    // Add client to the list of clients
    {
      std::scoped_lock lock{mut};
      m_clients.push_back(client);
    }
  }
  else if (sBuf.find("Bye") == 0)
  {
    // Disconnect
    {
      std::scoped_lock lock{mut};
      m_clients.erase(std::remove(m_clients.begin(), m_clients.end(), client), m_clients.end());
    }

    std::cout << "RJ: Log Client Disconnected!" << std::endl;
  }
}
#endif // defined(_WIN32)

void UDPLogger::Log(uint8_t *data, size_t size)
{
  // shoutouts to memory safety
  if (bufsize + size > FLATBUFFER_SIZE)
  {
    FlushLogBuffer();
  }

  if (size > FLATBUFFER_SIZE)
  {
    std::cout << "RJ: Packet Too Big!" << std::endl;
  }

  assert(bufsize + size < FLATBUFFER_SIZE);
  memcpy(buf + bufsize, data, size);
  bufsize += size;
}

void UDPLogger::SetTitle(std::string str)
{
  title = str;
}

flatbuffers::Offset<rj::CTREMotorStatusFrame>
UDPLogger::GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, TalonFX &motor,
                          rj::StatusFrame &frameType)
{
  frameType = rj::StatusFrame::StatusFrame_CTREMotorStatusFrame;

  Faults faults;
  motor.GetFaults(faults);

  return rj::CreateCTREMotorStatusFrame(
      fbb,
      motor.GetFirmwareVersion(),
      motor.GetBaseID(),
      motor.GetDeviceID(),
      motor.GetOutputCurrent(),
      motor.GetBusVoltage(),
      motor.GetMotorOutputPercent(),
      motor.GetMotorOutputVoltage(),
      motor.GetTemperature(),
      motor.GetSelectedSensorPosition(),
      motor.GetSelectedSensorVelocity(),
      motor.GetClosedLoopError(),
      motor.GetIntegralAccumulator(),
      motor.GetErrorDerivative(),
      0, // TODO: only call this for valid modes. motor.GetClosedLoopTarget(),
      0, // TODO: only call this for valid modes.
         // motor.GetActiveTrajectoryPosition(),
      0, // TODO: only call this for valid modes.
         // motor.GetActiveTrajectoryVelocity(),
      0, // TODO: only call this for valid modes.
         // motor.GetActiveTrajectoryArbFeedFwd(),
      faults.ToBitfield(),
      motor.HasResetOccurred(),
      motor.GetLastError(),
      static_cast<int32_t>(motor.GetControlMode()),
      motor.GetStatorCurrent(),
      motor.GetSupplyCurrent(),
      motor.IsFwdLimitSwitchClosed(),
      motor.IsRevLimitSwitchClosed());
}

flatbuffers::Offset<rj::CTREMotorStatusFrame>
UDPLogger::GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, TalonSRX &motor,
                          rj::StatusFrame &frameType)
{
  frameType = rj::StatusFrame::StatusFrame_CTREMotorStatusFrame;

  Faults faults;
  motor.GetFaults(faults);

  return rj::CreateCTREMotorStatusFrame(
    fbb, motor.GetFirmwareVersion(),
    motor.GetBaseID(),
    motor.GetDeviceID(),
    motor.GetOutputCurrent(),
    motor.GetBusVoltage(),
    motor.GetMotorOutputPercent(),
    motor.GetMotorOutputVoltage(),
    motor.GetTemperature(),
    motor.GetSelectedSensorPosition(),
    motor.GetSelectedSensorVelocity(),
    motor.GetClosedLoopError(),
    motor.GetIntegralAccumulator(),
    motor.GetErrorDerivative(),
    0, // TODO: only call this for valid modes. motor.GetClosedLoopTarget(),
    0, // TODO: only call this for valid modes.
        // motor.GetActiveTrajectoryPosition(),
    0, // TODO: only call this for valid modes.
        // motor.GetActiveTrajectoryVelocity(),
    0, // TODO: only call this for valid modes.
        // motor.GetActiveTrajectoryArbFeedFwd(),
    faults.ToBitfield(),
    motor.HasResetOccurred(),
    motor.GetLastError(),
    static_cast<int32_t>(motor.GetControlMode()),
    motor.GetStatorCurrent(),
    motor.GetSupplyCurrent(),
    motor.IsFwdLimitSwitchClosed(),
    motor.IsRevLimitSwitchClosed()
  );
}

flatbuffers::Offset<rj::CTREMotorStatusFrame>
UDPLogger::GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, VictorSPX &motor,
                          rj::StatusFrame &frameType)
{
  frameType = rj::StatusFrame::StatusFrame_CTREMotorStatusFrame;

  Faults faults;
  motor.GetFaults(faults);

  return rj::CreateCTREMotorStatusFrame(
    fbb, 
    motor.GetFirmwareVersion(), 
    motor.GetBaseID(), 
    motor.GetDeviceID(),
    0.0, 
    motor.GetBusVoltage(), 
    motor.GetMotorOutputPercent(),
    motor.GetMotorOutputVoltage(),
    motor.GetTemperature(),
    motor.GetSelectedSensorPosition(), 
    motor.GetSelectedSensorVelocity(),
    motor.GetClosedLoopError(), 
    motor.GetIntegralAccumulator(),
    motor.GetErrorDerivative(),
    0, // TODO: only call this for valid modes. motor.GetClosedLoopTarget(),
    0, // TODO: only call this for valid modes.
        // motor.GetActiveTrajectoryPosition(),
    0, // TODO: only call this for valid modes.
        // motor.GetActiveTrajectoryVelocity(),
    0, // TODO: only call this for valid modes.
        // motor.GetActiveTrajectoryArbFeedFwd(),
    faults.ToBitfield(), 
    motor.HasResetOccurred(), 
    motor.GetLastError(),
    static_cast<int32_t>(motor.GetControlMode()), 
    0.0, 
    0.0, 
    0, 
    0
  );
}
/*
rj::REVCANEncoder UDPLogger::GetREVCANEncoder(rev::CANEncoder &encoder) {
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

rj::REVCANAnalog UDPLogger::GetREVCANAnalog(rev::CANAnalog &analog) {
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

rj::REVPIDController UDPLogger::GetREVPIDController(rev::CANPIDController &pid,
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

rj::REVCANDigitalInput UDPLogger::GetREVCANDigitalInput(rev::CANDigitalInput &input, 
                          rj::StatusFrame &frameType) {
  return rj::REVCANDigitalInput(input.Get(), input.IsLimitSwitchEnabled());
}

flatbuffers::Offset<rj::REVMotorStatusFrame>
UDPLogger::GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, rev::CANSparkMax &motor) {
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

  auto revFwdLimit = GetREVCANDigitalInput(fwdLimit);
  auto revRevLimit = GetREVCANDigitalInput(revLimit);

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
*/

flatbuffers::Offset<rj::PDPStatusFrame>
UDPLogger::GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb,
                          frc::PowerDistributionPanel &device,
                          rj::StatusFrame &frameType)
{
  frameType = rj::StatusFrame::StatusFrame_PDPStatusFrame;

  std::vector<double> currentMeasurements{
      device.GetCurrent(0), device.GetCurrent(1), device.GetCurrent(2),
      device.GetCurrent(3), device.GetCurrent(4), device.GetCurrent(5),
      device.GetCurrent(6), device.GetCurrent(7), device.GetCurrent(8),
      device.GetCurrent(9), device.GetCurrent(10), device.GetCurrent(11),
      device.GetCurrent(12), device.GetCurrent(13), device.GetCurrent(14),
      device.GetCurrent(15)};

  return rj::CreatePDPStatusFrameDirect(
    fbb, 
    device.GetModule(), 
    device.GetVoltage(), 
    device.GetTemperature(),
    &currentMeasurements, 
    device.GetTotalCurrent(), 
    device.GetTotalPower(),
    device.GetTotalEnergy()
  );
}

flatbuffers::Offset<rj::PCMStatusFrame>
UDPLogger::GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, frc::Compressor &device,
                          rj::StatusFrame &frameType)
{
  frameType = rj::StatusFrame::StatusFrame_PCMStatusFrame;

  return rj::CreatePCMStatusFrame(
    fbb, 
    device.GetModule(), 
    device.Enabled(), 
    device.GetPressureSwitchValue(),
    device.GetCompressorCurrent(), 
    device.GetClosedLoopControl(),
    device.GetCompressorCurrentTooHighFault(),
    device.GetCompressorShortedFault(),
    device.GetCompressorNotConnectedFault()
  );
}
/*
rj::RawColor UDPLogger::GetRawColor(rev::ColorSensorV3::RawColor &color, 
                          rj::StatusFrame &frameType) {
  return rj::RawColor(color.red, color.green, color.blue, color.ir);
}

flatbuffers::Offset<rj::REVColorSensorStatusFrame>
UDPLogger::GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb,
               rev::ColorSensorV3 &colorSensor, 
                          rj::StatusFrame &frameType) {
  auto color = colorSensor.GetRawColor();
  auto revColor = GetRawColor(color);
  return rj::CreateREVColorSensorStatusFrame(
      fbb, &revColor, colorSensor.GetProximity(), colorSensor.HasReset());
}

rj::BoardYawAxis UDPLogger::GetBoardYawAxis(AHRS::BoardYawAxis &yaw, 
                          rj::StatusFrame &frameType) {
  return rj::BoardYawAxis(yaw.board_axis, yaw.up);
}

flatbuffers::Offset<rj::NavXStatusFrame>
UDPLogger::GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, AHRS &navx, 
                          rj::StatusFrame &frameType) {
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
*/

#ifdef __FRC_ROBORIO__
flatbuffers::Offset<rj::ADIS16470StatusFrame>
UDPLogger::GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, frc::ADIS16470_IMU &imu,
                          rj::StatusFrame &frameType)
{
  frameType = rj::StatusFrame::StatusFrame_ADIS16470StatusFrame;

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
      imu.GetYawAxis());
}
#endif

flatbuffers::Offset<rj::WPIDigitalInputStatusFrame>
UDPLogger::GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, frc::DigitalInput &input,
                          rj::StatusFrame &frameType)
{
  frameType = rj::StatusFrame::StatusFrame_WPIDigitalInputStatusFrame;

  return rj::CreateWPIDigitalInputStatusFrame(
      fbb,
      input.GetChannel(),
      input.Get(),
      input.IsAnalogTrigger());
}

flatbuffers::Offset<rj::WPIEncoderStatusFrame>
UDPLogger::GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, frc::Encoder &encoder,
                          rj::StatusFrame &frameType)
{
  frameType = rj::StatusFrame::StatusFrame_WPIEncoderStatusFrame;

  return rj::CreateWPIEncoderStatusFrame(
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
      encoder.GetFPGAIndex());
}

flatbuffers::Offset<rj::WPIDutyCycleEncoderStatusFrame>
UDPLogger::GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb,
                          frc::DutyCycleEncoder &encoder,
                          rj::StatusFrame &frameType)
{
  frameType = rj::StatusFrame::StatusFrame_WPIDutyCycleEncoderStatusFrame;

  return rj::CreateWPIDutyCycleEncoderStatusFrame(
      fbb,
      encoder.GetFrequency(),
      encoder.IsConnected(),
      (double)encoder.Get(),
      encoder.GetDistancePerRotation(),
      encoder.GetDistance(),
      encoder.GetFPGAIndex(),
      encoder.GetSourceChannel());
}
