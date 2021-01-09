#include "ExternalDeviceProvider.hpp"

#include "time.h"
#include <cmath>
#include <cstdlib>

flatbuffers::Offset<rj::CTREMotorStatusFrame>
GetStatusFrame(flatbuffers::FlatBufferBuilder& fbb, TalonFX& motor)
{
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
GetStatusFrame(flatbuffers::FlatBufferBuilder& fbb, TalonSRX& motor)
{
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
GetStatusFrame(flatbuffers::FlatBufferBuilder& fbb, VictorSPX& motor)
{
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
    0);
}

flatbuffers::Offset<rj::PDPStatusFrame>
GetStatusFrame(flatbuffers::FlatBufferBuilder& fbb,
               frc::PowerDistributionPanel& pdp)
{
  std::vector<double> currentMeasurements{
    pdp.GetCurrent(0),  pdp.GetCurrent(1),  pdp.GetCurrent(2),
    pdp.GetCurrent(3),  pdp.GetCurrent(4),  pdp.GetCurrent(5),
    pdp.GetCurrent(6),  pdp.GetCurrent(7),  pdp.GetCurrent(8),
    pdp.GetCurrent(9),  pdp.GetCurrent(10), pdp.GetCurrent(11),
    pdp.GetCurrent(12), pdp.GetCurrent(13), pdp.GetCurrent(14),
    pdp.GetCurrent(15)
  };

  return rj::CreatePDPStatusFrameDirect(fbb,
                                        pdp.GetModule(),
                                        pdp.GetVoltage(),
                                        pdp.GetTemperature(),
                                        &currentMeasurements,
                                        pdp.GetTotalCurrent(),
                                        pdp.GetTotalPower(),
                                        pdp.GetTotalEnergy());
}

flatbuffers::Offset<rj::PCMStatusFrame>
GetStatusFrame(flatbuffers::FlatBufferBuilder& fbb, frc::Compressor& pcm)
{
  return rj::CreatePCMStatusFrame(fbb,
                                  pcm.GetModule(),
                                  pcm.Enabled(),
                                  pcm.GetPressureSwitchValue(),
                                  pcm.GetCompressorCurrent(),
                                  pcm.GetClosedLoopControl(),
                                  pcm.GetCompressorCurrentTooHighFault(),
                                  pcm.GetCompressorShortedFault(),
                                  pcm.GetCompressorNotConnectedFault());
}

void
BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb, TalonFX& motor)
{
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, motor);

  auto statusFrameHolder = rj::CreateStatusFrameHolder(
    fbb,
    unixTime,
    monotonicTime,
    rj::StatusFrame::StatusFrame_CTREMotorStatusFrame,
    statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void
BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb, TalonSRX& motor)
{
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, motor);

  auto statusFrameHolder = rj::CreateStatusFrameHolder(
    fbb,
    unixTime,
    monotonicTime,
    rj::StatusFrame::StatusFrame_CTREMotorStatusFrame,
    statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void
BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb, VictorSPX& motor)
{
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, motor);

  auto statusFrameHolder = rj::CreateStatusFrameHolder(
    fbb,
    unixTime,
    monotonicTime,
    rj::StatusFrame::StatusFrame_CTREMotorStatusFrame,
    statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void
BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb,
                         frc::PowerDistributionPanel& pdp)
{
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, pdp);

  auto statusFrameHolder =
    rj::CreateStatusFrameHolder(fbb,
                                unixTime,
                                monotonicTime,
                                rj::StatusFrame::StatusFrame_PDPStatusFrame,
                                statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void
BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb,
                         frc::Compressor& pcm)
{
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, pcm);

  auto statusFrameHolder =
    rj::CreateStatusFrameHolder(fbb,
                                unixTime,
                                monotonicTime,
                                rj::StatusFrame::StatusFrame_PCMStatusFrame,
                                statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void
ExternalDeviceProvider::PopulateLogBuffer(UDPLogger& logger)
{
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
  // fbb.Reset();
  // BuildExternalDeviceFrame(fbb, navx);
  // buffer = fbb.Release();
  // logger.Log(buffer.data(), buffer.size());

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

  // TODO: Rev Color Sensor
  // fbb.Reset();
  // BuildExternalDeviceFrame(fbb, m_colorSensor);
  // buffer = fbb.Release();
  // logger.Log(buffer.data(), buffer.size());

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

  // TODO: Spark MAX
  // fbb.Reset();
  // BuildExternalDeviceFrame(fbb, sparkIndexerB);
  // buffer = fbb.Release();
  // logger.Log(buffer.data(), buffer.size());
  //
  // fbb.Reset();
  // BuildExternalDeviceFrame(fbb, sparkIndexerC);
  // buffer = fbb.Release();
  // logger.Log(buffer.data(), buffer.size());

  fbb.Reset();
  BuildExternalDeviceFrame(fbb, motorFeeder);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());

  fbb.Reset();
  BuildExternalDeviceFrame(fbb, motorHood);
  buffer = fbb.Release();
  logger.Log(buffer.data(), buffer.size());
}
