#pragma once

#include <functional>
#include <mutex>
#include <vector>

#if defined(_WIN32)

#else
#include <netinet/in.h>
#endif // defined(_WIN32)

#include "rev/CANSparkMax.h"
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

#define FLATBUFFER_SIZE 8192

class UDPLogger
{
private:
  flatbuffers::FlatBufferBuilder fbb{ FLATBUFFER_SIZE };
  uint8_t buf[FLATBUFFER_SIZE]; // 4KB
  size_t bufsize;

  int sockfd;
#if defined(_WIN32)
#else
  struct sockaddr_in address;
  std::vector<struct sockaddr_in> clients;
#endif // defined(_WIN32)
  std::recursive_mutex mut;
  std::string title;

  flatbuffers::Offset<rj::CTREMotorStatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder& fbb, ctre::phoenix::motorcontrol::can::TalonFX& pcm);
  flatbuffers::Offset<rj::CTREMotorStatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder& fbb, ctre::phoenix::motorcontrol::can::TalonSRX& pcm);
  flatbuffers::Offset<rj::CTREMotorStatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder& fbb, ctre::phoenix::motorcontrol::can::VictorSPX& pcm);

  rj::REVCANEncoder GetREVCANEncoder(rev::CANEncoder& pcm);
  rj::REVCANAnalog GetREVCANAnalog(rev::CANAnalog& pcm);
  rj::REVPIDController GetREVPIDController(rev::CANPIDController& pcm, int slotID);
  rj::REVCANDigitalInput GetREVCANDigitalInput(rev::CANDigitalInput& pcm);
  flatbuffers::Offset<rj::REVMotorStatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder& fbb, rev::CANSparkMax &motor);
  
  flatbuffers::Offset<rj::PDPStatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder& fbb, frc::PowerDistributionPanel& pcm);
  flatbuffers::Offset<rj::PCMStatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder& fbb, frc::Compressor& pcm);

  // rj::RawColor GetRawColor(rev::ColorSensorV3::RawColor &color);
  // flatbuffers::Offset<rj::REVColorSensorStatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, rev::ColorSensorV3 &colorSensor);

  // rj::BoardYawAxis GetBoardYawAxis(AHRS::BoardYawAxis &yaw);
  // flatbuffers::Offset<rj::NavXStatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, AHRS &navx);

  flatbuffers::Offset<rj::WPIDigitalInputStatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, frc::DigitalInput &input);
  flatbuffers::Offset<rj::WPIEncoderStatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, frc::Encoder &encoder);
  flatbuffers::Offset<rj::WPIDutyCycleEncoderStatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, frc::DutyCycleEncoder &encoder);
  flatbuffers::Offset<rj::CTRECanCoderStatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, ctre::phoenix::sensors::CANCoder &encoder);

  void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb, ctre::phoenix::motorcontrol::can::TalonFX& srx);
  void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb, ctre::phoenix::motorcontrol::can::TalonSRX& srx);
  void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb, ctre::phoenix::motorcontrol::can::VictorSPX& spx);
  void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb, frc::PowerDistributionPanel& pdp);
  void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb, frc::Compressor& pcm);
  void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb, rev::CANSparkMax& pcm);
  // void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb, rev::ColorSensorV3& pcm);
  // void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb, AHRS& pcm);
  void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb, frc::DigitalInput& pcm);
  void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb, frc::Encoder& pcm);
  void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb, frc::DutyCycleEncoder& pcm);
  void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb, ctre::phoenix::sensors::CANCoder& pcm);

#ifdef __FRC_ROBORIO__
  flatbuffers::Offset<rj::ADIS16470StatusFrame> GetStatusFrame(flatbuffers::FlatBufferBuilder &fbb, frc::ADIS16470_IMU &imu);
  void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb, frc::ADIS16470_IMU& pcm);
#endif

public:
  void InitLogger();
  void CheckForNewClient();
  void FlushLogBuffer();
  void Log(uint8_t* data, size_t size);
  void SetTitle(std::string str);

  void LogExternalDevice(ctre::phoenix::motorcontrol::can::TalonFX& fx);
  void LogExternalDevice(ctre::phoenix::motorcontrol::can::TalonSRX& srx);
  void LogExternalDevice(ctre::phoenix::motorcontrol::can::VictorSPX& spx);
  void LogExternalDevice(frc::PowerDistributionPanel& pdp);
  void LogExternalDevice(frc::Compressor& pcm);
  void LogExternalDevice(rev::CANSparkMax& pcm);
  // void LogExternalDevice(rev::ColorSensorV3& pcm);
  // void LogExternalDevice(AHRS& pcm);
  void LogExternalDevice(frc::DigitalInput& pcm);
  void LogExternalDevice(frc::Encoder& pcm);
  void LogExternalDevice(frc::DutyCycleEncoder& pcm);
  void LogExternalDevice(ctre::phoenix::sensors::CANCoder& pcm);

#ifdef __FRC_ROBORIO__
  void LogExternalDevice(frc::ADIS16470_IMU& pcm);
#endif
};
