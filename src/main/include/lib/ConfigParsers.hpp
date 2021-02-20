#pragma once
#include <lib/DrivebaseConfig.hpp>
#include <lib/ctreJsonSerde.hpp>

namespace rj {
inline void
to_json(wpi::json& nlohmann_json_j, const DrivebaseConfig& nlohmann_json_t)
{
  nlohmann_json_j["driveLeft1"] = nlohmann_json_t.driveLeft1;
  nlohmann_json_j["driveLeft2"] = nlohmann_json_t.driveLeft2;
  nlohmann_json_j["driveRight1"] = nlohmann_json_t.driveRight1;
  nlohmann_json_j["driveRight2"] = nlohmann_json_t.driveRight2;
  nlohmann_json_j["imu"] = nlohmann_json_t.imu;
  nlohmann_json_j["talonConfig"] = nlohmann_json_t.talonConfig;
  nlohmann_json_j["victorConfig"] = nlohmann_json_t.victorConfig;
  nlohmann_json_j["kSLinear"] = nlohmann_json_t.kSLinear;
  nlohmann_json_j["kVLinear"] = nlohmann_json_t.kVLinear;
  nlohmann_json_j["kALinear"] = nlohmann_json_t.kALinear;
  nlohmann_json_j["kSAngular"] = nlohmann_json_t.kSAngular;
  nlohmann_json_j["kVAngular"] = nlohmann_json_t.kVAngular;
  nlohmann_json_j["kAAngular"] = nlohmann_json_t.kAAngular;
}
inline void
from_json(const wpi::json& nlohmann_json_j, DrivebaseConfig& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.driveLeft1), void>::from_json(
    nlohmann_json_j.at("driveLeft1"), nlohmann_json_t.driveLeft1);
  wpi::adl_serializer<decltype(nlohmann_json_t.driveLeft2), void>::from_json(
    nlohmann_json_j.at("driveLeft2"), nlohmann_json_t.driveLeft2);
  wpi::adl_serializer<decltype(nlohmann_json_t.driveRight1), void>::from_json(
    nlohmann_json_j.at("driveRight1"), nlohmann_json_t.driveRight1);
  wpi::adl_serializer<decltype(nlohmann_json_t.driveRight2), void>::from_json(
    nlohmann_json_j.at("driveRight2"), nlohmann_json_t.driveRight2);
  wpi::adl_serializer<decltype(nlohmann_json_t.imu), void>::from_json(
    nlohmann_json_j.at("imu"), nlohmann_json_t.imu);
  wpi::adl_serializer<decltype(nlohmann_json_t.talonConfig), void>::from_json(
    nlohmann_json_j.at("talonConfig"), nlohmann_json_t.talonConfig);
  wpi::adl_serializer<decltype(nlohmann_json_t.victorConfig), void>::from_json(
    nlohmann_json_j.at("victorConfig"), nlohmann_json_t.victorConfig);
  wpi::adl_serializer<decltype(nlohmann_json_t.kSLinear), void>::from_json(
    nlohmann_json_j.at("kSLinear"), nlohmann_json_t.kSLinear);
  wpi::adl_serializer<decltype(nlohmann_json_t.kVLinear), void>::from_json(
    nlohmann_json_j.at("kVLinear"), nlohmann_json_t.kVLinear);
  wpi::adl_serializer<decltype(nlohmann_json_t.kALinear), void>::from_json(
    nlohmann_json_j.at("kALinear"), nlohmann_json_t.kALinear);
  wpi::adl_serializer<decltype(nlohmann_json_t.kSAngular), void>::from_json(
    nlohmann_json_j.at("kSAngular"), nlohmann_json_t.kSAngular);
  wpi::adl_serializer<decltype(nlohmann_json_t.kVAngular), void>::from_json(
    nlohmann_json_j.at("kVAngular"), nlohmann_json_t.kVAngular);
  wpi::adl_serializer<decltype(nlohmann_json_t.kAAngular), void>::from_json(
    nlohmann_json_j.at("kAAngular"), nlohmann_json_t.kAAngular);
}

inline void
to_json(wpi::json& nlohmann_json_j, const TalonSRXMap& nlohmann_json_t)
{
  nlohmann_json_j["id"] = nlohmann_json_t.id;
  nlohmann_json_j["invertType"] = nlohmann_json_t.invertType;
  nlohmann_json_j["neutralMode"] = nlohmann_json_t.neutralMode;
}
inline void
from_json(const wpi::json& nlohmann_json_j, TalonSRXMap& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.id), void>::from_json(
    nlohmann_json_j.at("id"), nlohmann_json_t.id);
  wpi::adl_serializer<decltype(nlohmann_json_t.invertType), void>::from_json(
    nlohmann_json_j.at("invertType"), nlohmann_json_t.invertType);
  wpi::adl_serializer<decltype(nlohmann_json_t.neutralMode), void>::from_json(
    nlohmann_json_j.at("neutralMode"), nlohmann_json_t.neutralMode);
}

inline void
to_json(wpi::json& nlohmann_json_j, const VictorSPXMap& nlohmann_json_t)
{
  nlohmann_json_j["id"] = nlohmann_json_t.id;
  nlohmann_json_j["invertType"] = nlohmann_json_t.invertType;
  nlohmann_json_j["neutralMode"] = nlohmann_json_t.neutralMode;
}
inline void
from_json(const wpi::json& nlohmann_json_j, VictorSPXMap& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.id), void>::from_json(
    nlohmann_json_j.at("id"), nlohmann_json_t.id);
  wpi::adl_serializer<decltype(nlohmann_json_t.invertType), void>::from_json(
    nlohmann_json_j.at("invertType"), nlohmann_json_t.invertType);
  wpi::adl_serializer<decltype(nlohmann_json_t.neutralMode), void>::from_json(
    nlohmann_json_j.at("neutralMode"), nlohmann_json_t.neutralMode);
}

inline void
to_json(wpi::json& nlohmann_json_j, const ADIS16470Map& nlohmann_json_t)
{
  nlohmann_json_j["yaw_axis"] = nlohmann_json_t.yaw_axis;
  nlohmann_json_j["port"] = nlohmann_json_t.port;
  nlohmann_json_j["cal_time"] = nlohmann_json_t.cal_time;
}
inline void
from_json(const wpi::json& nlohmann_json_j, ADIS16470Map& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.yaw_axis), void>::from_json(
    nlohmann_json_j.at("yaw_axis"), nlohmann_json_t.yaw_axis);
  wpi::adl_serializer<decltype(nlohmann_json_t.port), void>::from_json(
    nlohmann_json_j.at("port"), nlohmann_json_t.port);
  wpi::adl_serializer<decltype(nlohmann_json_t.cal_time), void>::from_json(
    nlohmann_json_j.at("cal_time"), nlohmann_json_t.cal_time);
}

} // namespace rj

namespace frc {
template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const ADIS16470_IMU::IMUAxis& e)
{
  static_assert(std::is_enum<ADIS16470_IMU::IMUAxis>::value,
                "ADIS16470_IMU::IMUAxis"
                " must be an enum!");
  static const std::pair<ADIS16470_IMU::IMUAxis, BasicJsonType> m[] = {
    { ADIS16470_IMU::IMUAxis::kX, "kX" },
    { ADIS16470_IMU::IMUAxis::kY, "kY" },
    { ADIS16470_IMU::IMUAxis::kZ, "kZ" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<ADIS16470_IMU::IMUAxis, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.first == e; });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, ADIS16470_IMU::IMUAxis& e)
{
  static_assert(std::is_enum<ADIS16470_IMU::IMUAxis>::value,
                "ADIS16470_IMU::IMUAxis"
                " must be an enum!");
  static const std::pair<ADIS16470_IMU::IMUAxis, BasicJsonType> m[] = {
    { ADIS16470_IMU::IMUAxis::kX, "kX" },
    { ADIS16470_IMU::IMUAxis::kY, "kY" },
    { ADIS16470_IMU::IMUAxis::kZ, "kZ" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<ADIS16470_IMU::IMUAxis, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.second == j; });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const SPI::Port& e)
{
  static_assert(std::is_enum<SPI::Port>::value,
                "SPI::Port"
                " must be an enum!");
  static const std::pair<SPI::Port, BasicJsonType> m[] = {
    { SPI::Port::kOnboardCS0, "kOnboardCS0" },
    { SPI::Port::kOnboardCS1, "kOnboardCS1" },
    { SPI::Port::kOnboardCS2, "kOnboardCS2" },
    { SPI::Port::kOnboardCS3, "kOnboardCS3" },
    { SPI::Port::kMXP, "kMXP" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<SPI::Port, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, SPI::Port& e)
{
  static_assert(std::is_enum<SPI::Port>::value,
                "SPI::Port"
                " must be an enum!");
  static const std::pair<SPI::Port, BasicJsonType> m[] = {
    { SPI::Port::kOnboardCS0, "kOnboardCS0" },
    { SPI::Port::kOnboardCS1, "kOnboardCS1" },
    { SPI::Port::kOnboardCS2, "kOnboardCS2" },
    { SPI::Port::kOnboardCS3, "kOnboardCS3" },
    { SPI::Port::kMXP, "kMXP" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<SPI::Port, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const ADIS16470CalibrationTime& e)
{
  static_assert(std::is_enum<ADIS16470CalibrationTime>::value,
                "ADIS16470CalibrationTime"
                " must be an enum!");
  static const std::pair<ADIS16470CalibrationTime, BasicJsonType> m[] = {
    { ADIS16470CalibrationTime::_32ms, "_32ms" },
    { ADIS16470CalibrationTime::_64ms, "_64ms" },
    { ADIS16470CalibrationTime::_128ms, "_128ms" },
    { ADIS16470CalibrationTime::_256ms, "_256ms" },
    { ADIS16470CalibrationTime::_512ms, "_512ms" },
    { ADIS16470CalibrationTime::_1s, "_1s" },
    { ADIS16470CalibrationTime::_2s, "_2s" },
    { ADIS16470CalibrationTime::_4s, "_4s" },
    { ADIS16470CalibrationTime::_8s, "_8s" },
    { ADIS16470CalibrationTime::_16s, "_16s" },
    { ADIS16470CalibrationTime::_32s, "_32s" },
    { ADIS16470CalibrationTime::_64s, "_64s" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<ADIS16470CalibrationTime, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.first == e; });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, ADIS16470CalibrationTime& e)
{
  static_assert(std::is_enum<ADIS16470CalibrationTime>::value,
                "ADIS16470CalibrationTime"
                " must be an enum!");
  static const std::pair<ADIS16470CalibrationTime, BasicJsonType> m[] = {
    { ADIS16470CalibrationTime::_32ms, "_32ms" },
    { ADIS16470CalibrationTime::_64ms, "_64ms" },
    { ADIS16470CalibrationTime::_128ms, "_128ms" },
    { ADIS16470CalibrationTime::_256ms, "_256ms" },
    { ADIS16470CalibrationTime::_512ms, "_512ms" },
    { ADIS16470CalibrationTime::_1s, "_1s" },
    { ADIS16470CalibrationTime::_2s, "_2s" },
    { ADIS16470CalibrationTime::_4s, "_4s" },
    { ADIS16470CalibrationTime::_8s, "_8s" },
    { ADIS16470CalibrationTime::_16s, "_16s" },
    { ADIS16470CalibrationTime::_32s, "_32s" },
    { ADIS16470CalibrationTime::_64s, "_64s" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<ADIS16470CalibrationTime, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.second == j; });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

} // namespace frc