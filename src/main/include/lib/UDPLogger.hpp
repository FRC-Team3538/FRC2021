#pragma once

#include <functional>
#include <mutex>
#include <vector>

#if !defined(_WIN32)
#include <netinet/in.h>
#endif

// #include "rev/CANSparkMax.h"
// #include "rev/ColorSensorV3.h"
// #include <AHRS.h>
#include <ctre/Phoenix.h>
#include <frc/Compressor.h>
#include <frc/PowerDistributionPanel.h>
#include <adi/ADIS16470_IMU.h>
#include <frc/Solenoid.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <frc/DigitalInput.h>

#include "flatbuffers/flatbuffers.h"
#include <proto/StatusFrame_generated.h>

using namespace std;

#define FLATBUFFER_SIZE 4096

class UDPLogger
{
private:
  uint m_udpPort = 3538;

  flatbuffers::FlatBufferBuilder fbb{FLATBUFFER_SIZE};
  uint8_t buf[FLATBUFFER_SIZE]; // 4KB
  size_t bufsize;

  int sockfd;
#if !defined(_WIN32)
  struct sockaddr_in address;
  std::vector<struct sockaddr_in> m_clients;
#endif
  std::mutex mut;
  std::string title;

  //
  // Status Frame Generators
  //
  flatbuffers::Offset<rj::CTREMotorStatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb,
                                                               ctre::phoenix::motorcontrol::can::TalonFX &device,
                                                               rj::StatusFrame &frameType);
  flatbuffers::Offset<rj::CTREMotorStatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb,
                                                               ctre::phoenix::motorcontrol::can::TalonSRX &device,
                                                               rj::StatusFrame &frameType);
  flatbuffers::Offset<rj::CTREMotorStatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb,
                                                               ctre::phoenix::motorcontrol::can::VictorSPX &device,
                                                               rj::StatusFrame &frameType);

  // rj::REVCANEncoder GetREVCANEncoder(rev::CANEncoder& device);
  // rj::REVCANAnalog GetREVCANAnalog(rev::CANAnalog& device);
  // rj::REVPIDController GetREVPIDController(rev::CANPIDController& device, int slotID);
  // rj::REVCANDigitalInput GetREVCANDigitalInput(rev::CANDigitalInput& device);
  // flatbuffers::Offset<rj::REVMotorStatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder& fbb, rev::CANSparkMax &motor, rj::StatusFrame);

  flatbuffers::Offset<rj::PDPStatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb,
                                                         frc::PowerDistributionPanel &device,
                                                         rj::StatusFrame &frameType);
  flatbuffers::Offset<rj::PCMStatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb,
                                                         frc::Compressor &device,
                                                         rj::StatusFrame &frameType);

  // rj::RawColor GetRawColor(rev::ColorSensorV3::RawColor &color);
  // flatbuffers::Offset<rj::REVColorSensorStatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, rev::ColorSensorV3 &colorSensor, rj::StatusFrame);

  // rj::BoardYawAxis GetBoardYawAxis(AHRS::BoardYawAxis &yaw);
  // flatbuffers::Offset<rj::NavXStatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, AHRS &navx, rj::StatusFrame);

  flatbuffers::Offset<rj::WPIDigitalInput> GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb,
                                                          frc::DigitalInput &input,
                                                          rj::StatusFrame &frameType);
  flatbuffers::Offset<rj::WPIEncoder> GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb,
                                                     frc::Encoder &encoder,
                                                     rj::StatusFrame &frameType);
  flatbuffers::Offset<rj::WPIDutyCycleEncoder> GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb,
                                                              frc::DutyCycleEncoder &encoder,
                                                              rj::StatusFrame &frameType);

#ifdef __FRC_ROBORIO__
  flatbuffers::Offset<rj::ADIS16470StatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb,
                                                               frc::ADIS16470_IMU &imu,
                                                               rj::StatusFrame &frameType);
#endif

  template <typename T>
  void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder &fbb,
                                           T &device)
  {
    // TODO(Dereck): The frame type should be determined from typeof(fbb)
    // This manual method is somewhat brittle
    rj::StatusFrame frameType;
    auto statusFrameOffset = GetStatusFrame(fbb, device, frameType);

    auto unixTime = frc::GetTime();
    auto monotonicTime = frc::Timer::GetFPGATimestamp();
    auto statusFrameHolder = rj::CreateStatusFrameHolder(
        fbb, unixTime, monotonicTime,
        frameType,
        statusFrameOffset.Union());

    rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
  }

public:
  void InitLogger();
  void CheckForNewClient();
  void FlushLogBuffer();
  void Log(uint8_t *data, size_t size);
  void SetTitle(std::string str);

  //
  // Logging ingest interface
  //
  template <typename T>
  void LogExternalDevice(T &device)
  {
    fbb.Reset();
    BuildExternalDeviceFrame(fbb, device);
    auto buffer = fbb.Release();
    Log(buffer.data(), buffer.size());
  }
};
