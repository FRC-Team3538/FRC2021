#pragma once
#include <ctre/Phoenix.h>
#include <wpi/json.h>

namespace ctre {
namespace phoenix {
template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const CANifierControlFrame& e)
{
  static_assert(std::is_enum<CANifierControlFrame>::value,
                "CANifierControlFrame"
                " must be an enum!");
  static const std::pair<CANifierControlFrame, BasicJsonType> m[] = {
    { CANifierControlFrame::CANifier_Control_1_General,
      "CANifier_Control_1_General" },
    { CANifierControlFrame::CANifier_Control_2_PwmOutput,
      "CANifier_Control_2_PwmOutput" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<CANifierControlFrame, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, CANifierControlFrame& e)
{
  static_assert(std::is_enum<CANifierControlFrame>::value,
                "CANifierControlFrame"
                " must be an enum!");
  static const std::pair<CANifierControlFrame, BasicJsonType> m[] = {
    { CANifierControlFrame::CANifier_Control_1_General,
      "CANifier_Control_1_General" },
    { CANifierControlFrame::CANifier_Control_2_PwmOutput,
      "CANifier_Control_2_PwmOutput" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<CANifierControlFrame, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.second == j; });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const CANifierStatusFrame& e)
{
  static_assert(std::is_enum<CANifierStatusFrame>::value,
                "CANifierStatusFrame"
                " must be an enum!");
  static const std::pair<CANifierStatusFrame, BasicJsonType> m[] = {
    { CANifierStatusFrame::CANifierStatusFrame_Status_1_General,
      "CANifierStatusFrame_Status_1_General" },
    { CANifierStatusFrame::CANifierStatusFrame_Status_2_General,
      "CANifierStatusFrame_Status_2_General" },
    { CANifierStatusFrame::CANifierStatusFrame_Status_3_PwmInputs0,
      "CANifierStatusFrame_Status_3_PwmInputs0" },
    { CANifierStatusFrame::CANifierStatusFrame_Status_4_PwmInputs1,
      "CANifierStatusFrame_Status_4_PwmInputs1" },
    { CANifierStatusFrame::CANifierStatusFrame_Status_5_PwmInputs2,
      "CANifierStatusFrame_Status_5_PwmInputs2" },
    { CANifierStatusFrame::CANifierStatusFrame_Status_6_PwmInputs3,
      "CANifierStatusFrame_Status_6_PwmInputs3" },
    { CANifierStatusFrame::CANifierStatusFrame_Status_8_Misc,
      "CANifierStatusFrame_Status_8_Misc" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<CANifierStatusFrame, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, CANifierStatusFrame& e)
{
  static_assert(std::is_enum<CANifierStatusFrame>::value,
                "CANifierStatusFrame"
                " must be an enum!");
  static const std::pair<CANifierStatusFrame, BasicJsonType> m[] = {
    { CANifierStatusFrame::CANifierStatusFrame_Status_1_General,
      "CANifierStatusFrame_Status_1_General" },
    { CANifierStatusFrame::CANifierStatusFrame_Status_2_General,
      "CANifierStatusFrame_Status_2_General" },
    { CANifierStatusFrame::CANifierStatusFrame_Status_3_PwmInputs0,
      "CANifierStatusFrame_Status_3_PwmInputs0" },
    { CANifierStatusFrame::CANifierStatusFrame_Status_4_PwmInputs1,
      "CANifierStatusFrame_Status_4_PwmInputs1" },
    { CANifierStatusFrame::CANifierStatusFrame_Status_5_PwmInputs2,
      "CANifierStatusFrame_Status_5_PwmInputs2" },
    { CANifierStatusFrame::CANifierStatusFrame_Status_6_PwmInputs3,
      "CANifierStatusFrame_Status_6_PwmInputs3" },
    { CANifierStatusFrame::CANifierStatusFrame_Status_8_Misc,
      "CANifierStatusFrame_Status_8_Misc" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<CANifierStatusFrame, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const CANifierVelocityMeasPeriod& e)
{
  static_assert(std::is_enum<CANifierVelocityMeasPeriod>::value,
                "CANifierVelocityMeasPeriod"
                " must be an enum!");
  static const std::pair<CANifierVelocityMeasPeriod, BasicJsonType> m[] = {
    { CANifierVelocityMeasPeriod::Period_1Ms, "Period_1Ms" },
    { CANifierVelocityMeasPeriod::Period_2Ms, "Period_2Ms" },
    { CANifierVelocityMeasPeriod::Period_5Ms, "Period_5Ms" },
    { CANifierVelocityMeasPeriod::Period_10Ms, "Period_10Ms" },
    { CANifierVelocityMeasPeriod::Period_20Ms, "Period_20Ms" },
    { CANifierVelocityMeasPeriod::Period_25Ms, "Period_25Ms" },
    { CANifierVelocityMeasPeriod::Period_50Ms, "Period_50Ms" },
    { CANifierVelocityMeasPeriod::Period_100Ms, "Period_100Ms" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<CANifierVelocityMeasPeriod, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.first == e; });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, CANifierVelocityMeasPeriod& e)
{
  static_assert(std::is_enum<CANifierVelocityMeasPeriod>::value,
                "CANifierVelocityMeasPeriod"
                " must be an enum!");
  static const std::pair<CANifierVelocityMeasPeriod, BasicJsonType> m[] = {
    { CANifierVelocityMeasPeriod::Period_1Ms, "Period_1Ms" },
    { CANifierVelocityMeasPeriod::Period_2Ms, "Period_2Ms" },
    { CANifierVelocityMeasPeriod::Period_5Ms, "Period_5Ms" },
    { CANifierVelocityMeasPeriod::Period_10Ms, "Period_10Ms" },
    { CANifierVelocityMeasPeriod::Period_20Ms, "Period_20Ms" },
    { CANifierVelocityMeasPeriod::Period_25Ms, "Period_25Ms" },
    { CANifierVelocityMeasPeriod::Period_50Ms, "Period_50Ms" },
    { CANifierVelocityMeasPeriod::Period_100Ms, "Period_100Ms" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<CANifierVelocityMeasPeriod, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.second == j; });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const ParamEnum& e)
{
  static_assert(std::is_enum<ParamEnum>::value,
                "ParamEnum"
                " must be an enum!");
  static const std::pair<ParamEnum, BasicJsonType> m[] = {
    { ParamEnum::eOnBoot_BrakeMode, "eOnBoot_BrakeMode" },
    { ParamEnum::eQuadFilterEn, "eQuadFilterEn" },
    { ParamEnum::eQuadIdxPolarity, "eQuadIdxPolarity" },
    { ParamEnum::eMotionProfileHasUnderrunErr, "eMotionProfileHasUnderrunErr" },
    { ParamEnum::eMotionProfileTrajectoryPointDurationMs,
      "eMotionProfileTrajectoryPointDurationMs" },
    { ParamEnum::eMotionProfileTrajectoryInterpolDis,
      "eMotionProfileTrajectoryInterpolDis" },
    { ParamEnum::eStatusFramePeriod, "eStatusFramePeriod" },
    { ParamEnum::eOpenloopRamp, "eOpenloopRamp" },
    { ParamEnum::eClosedloopRamp, "eClosedloopRamp" },
    { ParamEnum::eNeutralDeadband, "eNeutralDeadband" },
    { ParamEnum::ePeakPosOutput, "ePeakPosOutput" },
    { ParamEnum::eNominalPosOutput, "eNominalPosOutput" },
    { ParamEnum::ePeakNegOutput, "ePeakNegOutput" },
    { ParamEnum::eNominalNegOutput, "eNominalNegOutput" },
    { ParamEnum::eProfileParamSlot_P, "eProfileParamSlot_P" },
    { ParamEnum::eProfileParamSlot_I, "eProfileParamSlot_I" },
    { ParamEnum::eProfileParamSlot_D, "eProfileParamSlot_D" },
    { ParamEnum::eProfileParamSlot_F, "eProfileParamSlot_F" },
    { ParamEnum::eProfileParamSlot_IZone, "eProfileParamSlot_IZone" },
    { ParamEnum::eProfileParamSlot_AllowableErr,
      "eProfileParamSlot_AllowableErr" },
    { ParamEnum::eProfileParamSlot_MaxIAccum, "eProfileParamSlot_MaxIAccum" },
    { ParamEnum::eProfileParamSlot_PeakOutput, "eProfileParamSlot_PeakOutput" },
    { ParamEnum::eClearPositionOnLimitF, "eClearPositionOnLimitF" },
    { ParamEnum::eClearPositionOnLimitR, "eClearPositionOnLimitR" },
    { ParamEnum::eClearPositionOnQuadIdx, "eClearPositionOnQuadIdx" },
    { ParamEnum::eSampleVelocityPeriod, "eSampleVelocityPeriod" },
    { ParamEnum::eSampleVelocityWindow, "eSampleVelocityWindow" },
    { ParamEnum::eFeedbackSensorType, "eFeedbackSensorType" },
    { ParamEnum::eSelectedSensorPosition, "eSelectedSensorPosition" },
    { ParamEnum::eFeedbackNotContinuous, "eFeedbackNotContinuous" },
    { ParamEnum::eRemoteSensorSource, "eRemoteSensorSource" },
    { ParamEnum::eRemoteSensorDeviceID, "eRemoteSensorDeviceID" },
    { ParamEnum::eSensorTerm, "eSensorTerm" },
    { ParamEnum::eRemoteSensorClosedLoopDisableNeutralOnLOS,
      "eRemoteSensorClosedLoopDisableNeutralOnLOS" },
    { ParamEnum::ePIDLoopPolarity, "ePIDLoopPolarity" },
    { ParamEnum::ePIDLoopPeriod, "ePIDLoopPeriod" },
    { ParamEnum::eSelectedSensorCoefficient, "eSelectedSensorCoefficient" },
    { ParamEnum::eForwardSoftLimitThreshold, "eForwardSoftLimitThreshold" },
    { ParamEnum::eReverseSoftLimitThreshold, "eReverseSoftLimitThreshold" },
    { ParamEnum::eForwardSoftLimitEnable, "eForwardSoftLimitEnable" },
    { ParamEnum::eReverseSoftLimitEnable, "eReverseSoftLimitEnable" },
    { ParamEnum::eNominalBatteryVoltage, "eNominalBatteryVoltage" },
    { ParamEnum::eBatteryVoltageFilterSize, "eBatteryVoltageFilterSize" },
    { ParamEnum::eContinuousCurrentLimitAmps, "eContinuousCurrentLimitAmps" },
    { ParamEnum::ePeakCurrentLimitMs, "ePeakCurrentLimitMs" },
    { ParamEnum::ePeakCurrentLimitAmps, "ePeakCurrentLimitAmps" },
    { ParamEnum::eCurrLimit_Amps, "eCurrLimit_Amps" },
    { ParamEnum::eCurrThres_Amps, "eCurrThres_Amps" },
    { ParamEnum::eCurrEnable, "eCurrEnable" },
    { ParamEnum::eCurrThres_Ms, "eCurrThres_Ms" },
    { ParamEnum::eClosedLoopIAccum, "eClosedLoopIAccum" },
    { ParamEnum::eCustomParam, "eCustomParam" },
    { ParamEnum::eStickyFaults, "eStickyFaults" },
    { ParamEnum::eAnalogPosition, "eAnalogPosition" },
    { ParamEnum::eQuadraturePosition, "eQuadraturePosition" },
    { ParamEnum::ePulseWidthPosition, "ePulseWidthPosition" },
    { ParamEnum::eIntegratedSensor, "eIntegratedSensor" },
    { ParamEnum::eMotMag_Accel, "eMotMag_Accel" },
    { ParamEnum::eMotMag_VelCruise, "eMotMag_VelCruise" },
    { ParamEnum::eMotMag_SCurveLevel, "eMotMag_SCurveLevel" },
    { ParamEnum::eLimitSwitchSource, "eLimitSwitchSource" },
    { ParamEnum::eLimitSwitchNormClosedAndDis, "eLimitSwitchNormClosedAndDis" },
    { ParamEnum::eLimitSwitchDisableNeutralOnLOS,
      "eLimitSwitchDisableNeutralOnLOS" },
    { ParamEnum::eLimitSwitchRemoteDevID, "eLimitSwitchRemoteDevID" },
    { ParamEnum::eSoftLimitDisableNeutralOnLOS,
      "eSoftLimitDisableNeutralOnLOS" },
    { ParamEnum::ePulseWidthPeriod_EdgesPerRot,
      "ePulseWidthPeriod_EdgesPerRot" },
    { ParamEnum::ePulseWidthPeriod_FilterWindowSz,
      "ePulseWidthPeriod_FilterWindowSz" },
    { ParamEnum::eYawOffset, "eYawOffset" },
    { ParamEnum::eCompassOffset, "eCompassOffset" },
    { ParamEnum::eBetaGain, "eBetaGain" },
    { ParamEnum::eEnableCompassFusion, "eEnableCompassFusion" },
    { ParamEnum::eGyroNoMotionCal, "eGyroNoMotionCal" },
    { ParamEnum::eEnterCalibration, "eEnterCalibration" },
    { ParamEnum::eFusedHeadingOffset, "eFusedHeadingOffset" },
    { ParamEnum::eStatusFrameRate, "eStatusFrameRate" },
    { ParamEnum::eAccumZ, "eAccumZ" },
    { ParamEnum::eTempCompDisable, "eTempCompDisable" },
    { ParamEnum::eMotionMeas_tap_threshX, "eMotionMeas_tap_threshX" },
    { ParamEnum::eMotionMeas_tap_threshY, "eMotionMeas_tap_threshY" },
    { ParamEnum::eMotionMeas_tap_threshZ, "eMotionMeas_tap_threshZ" },
    { ParamEnum::eMotionMeas_tap_count, "eMotionMeas_tap_count" },
    { ParamEnum::eMotionMeas_tap_time, "eMotionMeas_tap_time" },
    { ParamEnum::eMotionMeas_tap_time_multi, "eMotionMeas_tap_time_multi" },
    { ParamEnum::eMotionMeas_shake_reject_thresh,
      "eMotionMeas_shake_reject_thresh" },
    { ParamEnum::eMotionMeas_shake_reject_time,
      "eMotionMeas_shake_reject_time" },
    { ParamEnum::eMotionMeas_shake_reject_timeout,
      "eMotionMeas_shake_reject_timeout" },
    { ParamEnum::eUnitString, "eUnitString" },
    { ParamEnum::eFeedbackTimeBase, "eFeedbackTimeBase" },
    { ParamEnum::eDefaultConfig, "eDefaultConfig" },
    { ParamEnum::eFastWriteCount, "eFastWriteCount" },
    { ParamEnum::eWriteCount, "eWriteCount" },
    { ParamEnum::eReserved1, "eReserved1" },
    { ParamEnum::eMotorCommutation, "eMotorCommutation" },
    { ParamEnum::eSensorInitStrategy, "eSensorInitStrategy" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<ParamEnum, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, ParamEnum& e)
{
  static_assert(std::is_enum<ParamEnum>::value,
                "ParamEnum"
                " must be an enum!");
  static const std::pair<ParamEnum, BasicJsonType> m[] = {
    { ParamEnum::eOnBoot_BrakeMode, "eOnBoot_BrakeMode" },
    { ParamEnum::eQuadFilterEn, "eQuadFilterEn" },
    { ParamEnum::eQuadIdxPolarity, "eQuadIdxPolarity" },
    { ParamEnum::eMotionProfileHasUnderrunErr, "eMotionProfileHasUnderrunErr" },
    { ParamEnum::eMotionProfileTrajectoryPointDurationMs,
      "eMotionProfileTrajectoryPointDurationMs" },
    { ParamEnum::eMotionProfileTrajectoryInterpolDis,
      "eMotionProfileTrajectoryInterpolDis" },
    { ParamEnum::eStatusFramePeriod, "eStatusFramePeriod" },
    { ParamEnum::eOpenloopRamp, "eOpenloopRamp" },
    { ParamEnum::eClosedloopRamp, "eClosedloopRamp" },
    { ParamEnum::eNeutralDeadband, "eNeutralDeadband" },
    { ParamEnum::ePeakPosOutput, "ePeakPosOutput" },
    { ParamEnum::eNominalPosOutput, "eNominalPosOutput" },
    { ParamEnum::ePeakNegOutput, "ePeakNegOutput" },
    { ParamEnum::eNominalNegOutput, "eNominalNegOutput" },
    { ParamEnum::eProfileParamSlot_P, "eProfileParamSlot_P" },
    { ParamEnum::eProfileParamSlot_I, "eProfileParamSlot_I" },
    { ParamEnum::eProfileParamSlot_D, "eProfileParamSlot_D" },
    { ParamEnum::eProfileParamSlot_F, "eProfileParamSlot_F" },
    { ParamEnum::eProfileParamSlot_IZone, "eProfileParamSlot_IZone" },
    { ParamEnum::eProfileParamSlot_AllowableErr,
      "eProfileParamSlot_AllowableErr" },
    { ParamEnum::eProfileParamSlot_MaxIAccum, "eProfileParamSlot_MaxIAccum" },
    { ParamEnum::eProfileParamSlot_PeakOutput, "eProfileParamSlot_PeakOutput" },
    { ParamEnum::eClearPositionOnLimitF, "eClearPositionOnLimitF" },
    { ParamEnum::eClearPositionOnLimitR, "eClearPositionOnLimitR" },
    { ParamEnum::eClearPositionOnQuadIdx, "eClearPositionOnQuadIdx" },
    { ParamEnum::eSampleVelocityPeriod, "eSampleVelocityPeriod" },
    { ParamEnum::eSampleVelocityWindow, "eSampleVelocityWindow" },
    { ParamEnum::eFeedbackSensorType, "eFeedbackSensorType" },
    { ParamEnum::eSelectedSensorPosition, "eSelectedSensorPosition" },
    { ParamEnum::eFeedbackNotContinuous, "eFeedbackNotContinuous" },
    { ParamEnum::eRemoteSensorSource, "eRemoteSensorSource" },
    { ParamEnum::eRemoteSensorDeviceID, "eRemoteSensorDeviceID" },
    { ParamEnum::eSensorTerm, "eSensorTerm" },
    { ParamEnum::eRemoteSensorClosedLoopDisableNeutralOnLOS,
      "eRemoteSensorClosedLoopDisableNeutralOnLOS" },
    { ParamEnum::ePIDLoopPolarity, "ePIDLoopPolarity" },
    { ParamEnum::ePIDLoopPeriod, "ePIDLoopPeriod" },
    { ParamEnum::eSelectedSensorCoefficient, "eSelectedSensorCoefficient" },
    { ParamEnum::eForwardSoftLimitThreshold, "eForwardSoftLimitThreshold" },
    { ParamEnum::eReverseSoftLimitThreshold, "eReverseSoftLimitThreshold" },
    { ParamEnum::eForwardSoftLimitEnable, "eForwardSoftLimitEnable" },
    { ParamEnum::eReverseSoftLimitEnable, "eReverseSoftLimitEnable" },
    { ParamEnum::eNominalBatteryVoltage, "eNominalBatteryVoltage" },
    { ParamEnum::eBatteryVoltageFilterSize, "eBatteryVoltageFilterSize" },
    { ParamEnum::eContinuousCurrentLimitAmps, "eContinuousCurrentLimitAmps" },
    { ParamEnum::ePeakCurrentLimitMs, "ePeakCurrentLimitMs" },
    { ParamEnum::ePeakCurrentLimitAmps, "ePeakCurrentLimitAmps" },
    { ParamEnum::eCurrLimit_Amps, "eCurrLimit_Amps" },
    { ParamEnum::eCurrThres_Amps, "eCurrThres_Amps" },
    { ParamEnum::eCurrEnable, "eCurrEnable" },
    { ParamEnum::eCurrThres_Ms, "eCurrThres_Ms" },
    { ParamEnum::eClosedLoopIAccum, "eClosedLoopIAccum" },
    { ParamEnum::eCustomParam, "eCustomParam" },
    { ParamEnum::eStickyFaults, "eStickyFaults" },
    { ParamEnum::eAnalogPosition, "eAnalogPosition" },
    { ParamEnum::eQuadraturePosition, "eQuadraturePosition" },
    { ParamEnum::ePulseWidthPosition, "ePulseWidthPosition" },
    { ParamEnum::eIntegratedSensor, "eIntegratedSensor" },
    { ParamEnum::eMotMag_Accel, "eMotMag_Accel" },
    { ParamEnum::eMotMag_VelCruise, "eMotMag_VelCruise" },
    { ParamEnum::eMotMag_SCurveLevel, "eMotMag_SCurveLevel" },
    { ParamEnum::eLimitSwitchSource, "eLimitSwitchSource" },
    { ParamEnum::eLimitSwitchNormClosedAndDis, "eLimitSwitchNormClosedAndDis" },
    { ParamEnum::eLimitSwitchDisableNeutralOnLOS,
      "eLimitSwitchDisableNeutralOnLOS" },
    { ParamEnum::eLimitSwitchRemoteDevID, "eLimitSwitchRemoteDevID" },
    { ParamEnum::eSoftLimitDisableNeutralOnLOS,
      "eSoftLimitDisableNeutralOnLOS" },
    { ParamEnum::ePulseWidthPeriod_EdgesPerRot,
      "ePulseWidthPeriod_EdgesPerRot" },
    { ParamEnum::ePulseWidthPeriod_FilterWindowSz,
      "ePulseWidthPeriod_FilterWindowSz" },
    { ParamEnum::eYawOffset, "eYawOffset" },
    { ParamEnum::eCompassOffset, "eCompassOffset" },
    { ParamEnum::eBetaGain, "eBetaGain" },
    { ParamEnum::eEnableCompassFusion, "eEnableCompassFusion" },
    { ParamEnum::eGyroNoMotionCal, "eGyroNoMotionCal" },
    { ParamEnum::eEnterCalibration, "eEnterCalibration" },
    { ParamEnum::eFusedHeadingOffset, "eFusedHeadingOffset" },
    { ParamEnum::eStatusFrameRate, "eStatusFrameRate" },
    { ParamEnum::eAccumZ, "eAccumZ" },
    { ParamEnum::eTempCompDisable, "eTempCompDisable" },
    { ParamEnum::eMotionMeas_tap_threshX, "eMotionMeas_tap_threshX" },
    { ParamEnum::eMotionMeas_tap_threshY, "eMotionMeas_tap_threshY" },
    { ParamEnum::eMotionMeas_tap_threshZ, "eMotionMeas_tap_threshZ" },
    { ParamEnum::eMotionMeas_tap_count, "eMotionMeas_tap_count" },
    { ParamEnum::eMotionMeas_tap_time, "eMotionMeas_tap_time" },
    { ParamEnum::eMotionMeas_tap_time_multi, "eMotionMeas_tap_time_multi" },
    { ParamEnum::eMotionMeas_shake_reject_thresh,
      "eMotionMeas_shake_reject_thresh" },
    { ParamEnum::eMotionMeas_shake_reject_time,
      "eMotionMeas_shake_reject_time" },
    { ParamEnum::eMotionMeas_shake_reject_timeout,
      "eMotionMeas_shake_reject_timeout" },
    { ParamEnum::eUnitString, "eUnitString" },
    { ParamEnum::eFeedbackTimeBase, "eFeedbackTimeBase" },
    { ParamEnum::eDefaultConfig, "eDefaultConfig" },
    { ParamEnum::eFastWriteCount, "eFastWriteCount" },
    { ParamEnum::eWriteCount, "eWriteCount" },
    { ParamEnum::eReserved1, "eReserved1" },
    { ParamEnum::eMotorCommutation, "eMotorCommutation" },
    { ParamEnum::eSensorInitStrategy, "eSensorInitStrategy" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<ParamEnum, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const ErrorCode& e)
{
  static_assert(std::is_enum<ErrorCode>::value,
                "ErrorCode"
                " must be an enum!");
  static const std::pair<ErrorCode, BasicJsonType> m[] = {
    { ErrorCode::OK, "OK" },
    { ErrorCode::OKAY, "OKAY" },
    { ErrorCode::CAN_MSG_STALE, "CAN_MSG_STALE" },
    { ErrorCode::CAN_TX_FULL, "CAN_TX_FULL" },
    { ErrorCode::TxFailed, "TxFailed" },
    { ErrorCode::InvalidParamValue, "InvalidParamValue" },
    { ErrorCode::CAN_INVALID_PARAM, "CAN_INVALID_PARAM" },
    { ErrorCode::RxTimeout, "RxTimeout" },
    { ErrorCode::CAN_MSG_NOT_FOUND, "CAN_MSG_NOT_FOUND" },
    { ErrorCode::TxTimeout, "TxTimeout" },
    { ErrorCode::CAN_NO_MORE_TX_JOBS, "CAN_NO_MORE_TX_JOBS" },
    { ErrorCode::UnexpectedArbId, "UnexpectedArbId" },
    { ErrorCode::CAN_NO_SESSIONS_AVAIL, "CAN_NO_SESSIONS_AVAIL" },
    { ErrorCode::BufferFull, "BufferFull" },
    { ErrorCode::CAN_OVERFLOW, "CAN_OVERFLOW" },
    { ErrorCode::SensorNotPresent, "SensorNotPresent" },
    { ErrorCode::FirmwareTooOld, "FirmwareTooOld" },
    { ErrorCode::CouldNotChangePeriod, "CouldNotChangePeriod" },
    { ErrorCode::BufferFailure, "BufferFailure" },
    { ErrorCode::FirwmwareNonFRC, "FirwmwareNonFRC" },
    { ErrorCode::GeneralError, "GeneralError" },
    { ErrorCode::GENERAL_ERROR, "GENERAL_ERROR" },
    { ErrorCode::SIG_NOT_UPDATED, "SIG_NOT_UPDATED" },
    { ErrorCode::SigNotUpdated, "SigNotUpdated" },
    { ErrorCode::NotAllPIDValuesUpdated, "NotAllPIDValuesUpdated" },
    { ErrorCode::GEN_PORT_ERROR, "GEN_PORT_ERROR" },
    { ErrorCode::PORT_MODULE_TYPE_MISMATCH, "PORT_MODULE_TYPE_MISMATCH" },
    { ErrorCode::GEN_MODULE_ERROR, "GEN_MODULE_ERROR" },
    { ErrorCode::MODULE_NOT_INIT_SET_ERROR, "MODULE_NOT_INIT_SET_ERROR" },
    { ErrorCode::MODULE_NOT_INIT_GET_ERROR, "MODULE_NOT_INIT_GET_ERROR" },
    { ErrorCode::WheelRadiusTooSmall, "WheelRadiusTooSmall" },
    { ErrorCode::TicksPerRevZero, "TicksPerRevZero" },
    { ErrorCode::DistanceBetweenWheelsTooSmall,
      "DistanceBetweenWheelsTooSmall" },
    { ErrorCode::GainsAreNotSet, "GainsAreNotSet" },
    { ErrorCode::WrongRemoteLimitSwitchSource, "WrongRemoteLimitSwitchSource" },
    { ErrorCode::DoubleVoltageCompensatingWPI, "DoubleVoltageCompensatingWPI" },
    { ErrorCode::IncompatibleMode, "IncompatibleMode" },
    { ErrorCode::InvalidHandle, "InvalidHandle" },
    { ErrorCode::FeatureRequiresHigherFirm, "FeatureRequiresHigherFirm" },
    { ErrorCode::MotorControllerFeatureRequiresHigherFirm,
      "MotorControllerFeatureRequiresHigherFirm" },
    { ErrorCode::TalonFeatureRequiresHigherFirm,
      "TalonFeatureRequiresHigherFirm" },
    { ErrorCode::ConfigFactoryDefaultRequiresHigherFirm,
      "ConfigFactoryDefaultRequiresHigherFirm" },
    { ErrorCode::ConfigMotionSCurveRequiresHigherFirm,
      "ConfigMotionSCurveRequiresHigherFirm" },
    { ErrorCode::TalonFXFirmwarePreVBatDetect, "TalonFXFirmwarePreVBatDetect" },
    { ErrorCode::LibraryCouldNotBeLoaded, "LibraryCouldNotBeLoaded" },
    { ErrorCode::MissingRoutineInLibrary, "MissingRoutineInLibrary" },
    { ErrorCode::ResourceNotAvailable, "ResourceNotAvailable" },
    { ErrorCode::MusicFileNotFound, "MusicFileNotFound" },
    { ErrorCode::MusicFileWrongSize, "MusicFileWrongSize" },
    { ErrorCode::MusicFileTooNew, "MusicFileTooNew" },
    { ErrorCode::MusicFileInvalid, "MusicFileInvalid" },
    { ErrorCode::InvalidOrchestraAction, "InvalidOrchestraAction" },
    { ErrorCode::MusicFileTooOld, "MusicFileTooOld" },
    { ErrorCode::MusicInterrupted, "MusicInterrupted" },
    { ErrorCode::MusicNotSupported, "MusicNotSupported" },
    { ErrorCode::PulseWidthSensorNotPresent, "PulseWidthSensorNotPresent" },
    { ErrorCode::GeneralWarning, "GeneralWarning" },
    { ErrorCode::FeatureNotSupported, "FeatureNotSupported" },
    { ErrorCode::NotImplemented, "NotImplemented" },
    { ErrorCode::FirmVersionCouldNotBeRetrieved,
      "FirmVersionCouldNotBeRetrieved" },
    { ErrorCode::FeaturesNotAvailableYet, "FeaturesNotAvailableYet" },
    { ErrorCode::ControlModeNotValid, "ControlModeNotValid" },
    { ErrorCode::ControlModeNotSupportedYet, "ControlModeNotSupportedYet" },
    { ErrorCode::CascadedPIDNotSupporteYet, "CascadedPIDNotSupporteYet" },
    { ErrorCode::AuxiliaryPIDNotSupportedYet, "AuxiliaryPIDNotSupportedYet" },
    { ErrorCode::RemoteSensorsNotSupportedYet, "RemoteSensorsNotSupportedYet" },
    { ErrorCode::MotProfFirmThreshold, "MotProfFirmThreshold" },
    { ErrorCode::MotProfFirmThreshold2, "MotProfFirmThreshold2" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<ErrorCode, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, ErrorCode& e)
{
  static_assert(std::is_enum<ErrorCode>::value,
                "ErrorCode"
                " must be an enum!");
  static const std::pair<ErrorCode, BasicJsonType> m[] = {
    { ErrorCode::OK, "OK" },
    { ErrorCode::OKAY, "OKAY" },
    { ErrorCode::CAN_MSG_STALE, "CAN_MSG_STALE" },
    { ErrorCode::CAN_TX_FULL, "CAN_TX_FULL" },
    { ErrorCode::TxFailed, "TxFailed" },
    { ErrorCode::InvalidParamValue, "InvalidParamValue" },
    { ErrorCode::CAN_INVALID_PARAM, "CAN_INVALID_PARAM" },
    { ErrorCode::RxTimeout, "RxTimeout" },
    { ErrorCode::CAN_MSG_NOT_FOUND, "CAN_MSG_NOT_FOUND" },
    { ErrorCode::TxTimeout, "TxTimeout" },
    { ErrorCode::CAN_NO_MORE_TX_JOBS, "CAN_NO_MORE_TX_JOBS" },
    { ErrorCode::UnexpectedArbId, "UnexpectedArbId" },
    { ErrorCode::CAN_NO_SESSIONS_AVAIL, "CAN_NO_SESSIONS_AVAIL" },
    { ErrorCode::BufferFull, "BufferFull" },
    { ErrorCode::CAN_OVERFLOW, "CAN_OVERFLOW" },
    { ErrorCode::SensorNotPresent, "SensorNotPresent" },
    { ErrorCode::FirmwareTooOld, "FirmwareTooOld" },
    { ErrorCode::CouldNotChangePeriod, "CouldNotChangePeriod" },
    { ErrorCode::BufferFailure, "BufferFailure" },
    { ErrorCode::FirwmwareNonFRC, "FirwmwareNonFRC" },
    { ErrorCode::GeneralError, "GeneralError" },
    { ErrorCode::GENERAL_ERROR, "GENERAL_ERROR" },
    { ErrorCode::SIG_NOT_UPDATED, "SIG_NOT_UPDATED" },
    { ErrorCode::SigNotUpdated, "SigNotUpdated" },
    { ErrorCode::NotAllPIDValuesUpdated, "NotAllPIDValuesUpdated" },
    { ErrorCode::GEN_PORT_ERROR, "GEN_PORT_ERROR" },
    { ErrorCode::PORT_MODULE_TYPE_MISMATCH, "PORT_MODULE_TYPE_MISMATCH" },
    { ErrorCode::GEN_MODULE_ERROR, "GEN_MODULE_ERROR" },
    { ErrorCode::MODULE_NOT_INIT_SET_ERROR, "MODULE_NOT_INIT_SET_ERROR" },
    { ErrorCode::MODULE_NOT_INIT_GET_ERROR, "MODULE_NOT_INIT_GET_ERROR" },
    { ErrorCode::WheelRadiusTooSmall, "WheelRadiusTooSmall" },
    { ErrorCode::TicksPerRevZero, "TicksPerRevZero" },
    { ErrorCode::DistanceBetweenWheelsTooSmall,
      "DistanceBetweenWheelsTooSmall" },
    { ErrorCode::GainsAreNotSet, "GainsAreNotSet" },
    { ErrorCode::WrongRemoteLimitSwitchSource, "WrongRemoteLimitSwitchSource" },
    { ErrorCode::DoubleVoltageCompensatingWPI, "DoubleVoltageCompensatingWPI" },
    { ErrorCode::IncompatibleMode, "IncompatibleMode" },
    { ErrorCode::InvalidHandle, "InvalidHandle" },
    { ErrorCode::FeatureRequiresHigherFirm, "FeatureRequiresHigherFirm" },
    { ErrorCode::MotorControllerFeatureRequiresHigherFirm,
      "MotorControllerFeatureRequiresHigherFirm" },
    { ErrorCode::TalonFeatureRequiresHigherFirm,
      "TalonFeatureRequiresHigherFirm" },
    { ErrorCode::ConfigFactoryDefaultRequiresHigherFirm,
      "ConfigFactoryDefaultRequiresHigherFirm" },
    { ErrorCode::ConfigMotionSCurveRequiresHigherFirm,
      "ConfigMotionSCurveRequiresHigherFirm" },
    { ErrorCode::TalonFXFirmwarePreVBatDetect, "TalonFXFirmwarePreVBatDetect" },
    { ErrorCode::LibraryCouldNotBeLoaded, "LibraryCouldNotBeLoaded" },
    { ErrorCode::MissingRoutineInLibrary, "MissingRoutineInLibrary" },
    { ErrorCode::ResourceNotAvailable, "ResourceNotAvailable" },
    { ErrorCode::MusicFileNotFound, "MusicFileNotFound" },
    { ErrorCode::MusicFileWrongSize, "MusicFileWrongSize" },
    { ErrorCode::MusicFileTooNew, "MusicFileTooNew" },
    { ErrorCode::MusicFileInvalid, "MusicFileInvalid" },
    { ErrorCode::InvalidOrchestraAction, "InvalidOrchestraAction" },
    { ErrorCode::MusicFileTooOld, "MusicFileTooOld" },
    { ErrorCode::MusicInterrupted, "MusicInterrupted" },
    { ErrorCode::MusicNotSupported, "MusicNotSupported" },
    { ErrorCode::PulseWidthSensorNotPresent, "PulseWidthSensorNotPresent" },
    { ErrorCode::GeneralWarning, "GeneralWarning" },
    { ErrorCode::FeatureNotSupported, "FeatureNotSupported" },
    { ErrorCode::NotImplemented, "NotImplemented" },
    { ErrorCode::FirmVersionCouldNotBeRetrieved,
      "FirmVersionCouldNotBeRetrieved" },
    { ErrorCode::FeaturesNotAvailableYet, "FeaturesNotAvailableYet" },
    { ErrorCode::ControlModeNotValid, "ControlModeNotValid" },
    { ErrorCode::ControlModeNotSupportedYet, "ControlModeNotSupportedYet" },
    { ErrorCode::CascadedPIDNotSupporteYet, "CascadedPIDNotSupporteYet" },
    { ErrorCode::AuxiliaryPIDNotSupportedYet, "AuxiliaryPIDNotSupportedYet" },
    { ErrorCode::RemoteSensorsNotSupportedYet, "RemoteSensorsNotSupportedYet" },
    { ErrorCode::MotProfFirmThreshold, "MotProfFirmThreshold" },
    { ErrorCode::MotProfFirmThreshold2, "MotProfFirmThreshold2" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<ErrorCode, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const CANifier::LEDChannel& e)
{
  static_assert(std::is_enum<CANifier::LEDChannel>::value,
                "CANifier::LEDChannel"
                " must be an enum!");
  static const std::pair<CANifier::LEDChannel, BasicJsonType> m[] = {
    { CANifier::LEDChannel::LEDChannelA, "LEDChannelA" },
    { CANifier::LEDChannel::LEDChannelB, "LEDChannelB" },
    { CANifier::LEDChannel::LEDChannelC, "LEDChannelC" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<CANifier::LEDChannel, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, CANifier::LEDChannel& e)
{
  static_assert(std::is_enum<CANifier::LEDChannel>::value,
                "CANifier::LEDChannel"
                " must be an enum!");
  static const std::pair<CANifier::LEDChannel, BasicJsonType> m[] = {
    { CANifier::LEDChannel::LEDChannelA, "LEDChannelA" },
    { CANifier::LEDChannel::LEDChannelB, "LEDChannelB" },
    { CANifier::LEDChannel::LEDChannelC, "LEDChannelC" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<CANifier::LEDChannel, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.second == j; });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const CANifier::PWMChannel& e)
{
  static_assert(std::is_enum<CANifier::PWMChannel>::value,
                "CANifier::PWMChannel"
                " must be an enum!");
  static const std::pair<CANifier::PWMChannel, BasicJsonType> m[] = {
    { CANifier::PWMChannel::PWMChannel0, "PWMChannel0" },
    { CANifier::PWMChannel::PWMChannel1, "PWMChannel1" },
    { CANifier::PWMChannel::PWMChannel2, "PWMChannel2" },
    { CANifier::PWMChannel::PWMChannel3, "PWMChannel3" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<CANifier::PWMChannel, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, CANifier::PWMChannel& e)
{
  static_assert(std::is_enum<CANifier::PWMChannel>::value,
                "CANifier::PWMChannel"
                " must be an enum!");
  static const std::pair<CANifier::PWMChannel, BasicJsonType> m[] = {
    { CANifier::PWMChannel::PWMChannel0, "PWMChannel0" },
    { CANifier::PWMChannel::PWMChannel1, "PWMChannel1" },
    { CANifier::PWMChannel::PWMChannel2, "PWMChannel2" },
    { CANifier::PWMChannel::PWMChannel3, "PWMChannel3" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<CANifier::PWMChannel, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.second == j; });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const CANifier::GeneralPin& e)
{
  static_assert(std::is_enum<CANifier::GeneralPin>::value,
                "CANifier::GeneralPin"
                " must be an enum!");
  static const std::pair<CANifier::GeneralPin, BasicJsonType> m[] = {
    { CANifier::GeneralPin::QUAD_IDX, "QUAD_IDX" },
    { CANifier::GeneralPin::QUAD_B, "QUAD_B" },
    { CANifier::GeneralPin::QUAD_A, "QUAD_A" },
    { CANifier::GeneralPin::LIMR, "LIMR" },
    { CANifier::GeneralPin::LIMF, "LIMF" },
    { CANifier::GeneralPin::SDA, "SDA" },
    { CANifier::GeneralPin::SCL, "SCL" },
    { CANifier::GeneralPin::SPI_CS, "SPI_CS" },
    { CANifier::GeneralPin::SPI_MISO_PWM2P, "SPI_MISO_PWM2P" },
    { CANifier::GeneralPin::SPI_MOSI_PWM1P, "SPI_MOSI_PWM1P" },
    { CANifier::GeneralPin::SPI_CLK_PWM0P, "SPI_CLK_PWM0P" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<CANifier::GeneralPin, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, CANifier::GeneralPin& e)
{
  static_assert(std::is_enum<CANifier::GeneralPin>::value,
                "CANifier::GeneralPin"
                " must be an enum!");
  static const std::pair<CANifier::GeneralPin, BasicJsonType> m[] = {
    { CANifier::GeneralPin::QUAD_IDX, "QUAD_IDX" },
    { CANifier::GeneralPin::QUAD_B, "QUAD_B" },
    { CANifier::GeneralPin::QUAD_A, "QUAD_A" },
    { CANifier::GeneralPin::LIMR, "LIMR" },
    { CANifier::GeneralPin::LIMF, "LIMF" },
    { CANifier::GeneralPin::SDA, "SDA" },
    { CANifier::GeneralPin::SCL, "SCL" },
    { CANifier::GeneralPin::SPI_CS, "SPI_CS" },
    { CANifier::GeneralPin::SPI_MISO_PWM2P, "SPI_MISO_PWM2P" },
    { CANifier::GeneralPin::SPI_MOSI_PWM1P, "SPI_MOSI_PWM1P" },
    { CANifier::GeneralPin::SPI_CLK_PWM0P, "SPI_CLK_PWM0P" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<CANifier::GeneralPin, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.second == j; });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

namespace motorcontrol {
template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const NeutralMode& e)
{
  static_assert(std::is_enum<NeutralMode>::value,
                "NeutralMode"
                " must be an enum!");
  static const std::pair<NeutralMode, BasicJsonType> m[] = {
    { NeutralMode::EEPROMSetting, "EEPROMSetting" },
    { NeutralMode::Coast, "Coast" },
    { NeutralMode::Brake, "Brake" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<NeutralMode, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, NeutralMode& e)
{
  static_assert(std::is_enum<NeutralMode>::value,
                "NeutralMode"
                " must be an enum!");
  static const std::pair<NeutralMode, BasicJsonType> m[] = {
    { NeutralMode::EEPROMSetting, "EEPROMSetting" },
    { NeutralMode::Coast, "Coast" },
    { NeutralMode::Brake, "Brake" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<NeutralMode, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const VelocityMeasPeriod& e)
{
  static_assert(std::is_enum<VelocityMeasPeriod>::value,
                "VelocityMeasPeriod"
                " must be an enum!");
  static const std::pair<VelocityMeasPeriod, BasicJsonType> m[] = {
    { VelocityMeasPeriod::Period_1Ms, "Period_1Ms" },
    { VelocityMeasPeriod::Period_2Ms, "Period_2Ms" },
    { VelocityMeasPeriod::Period_5Ms, "Period_5Ms" },
    { VelocityMeasPeriod::Period_10Ms, "Period_10Ms" },
    { VelocityMeasPeriod::Period_20Ms, "Period_20Ms" },
    { VelocityMeasPeriod::Period_25Ms, "Period_25Ms" },
    { VelocityMeasPeriod::Period_50Ms, "Period_50Ms" },
    { VelocityMeasPeriod::Period_100Ms, "Period_100Ms" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<VelocityMeasPeriod, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, VelocityMeasPeriod& e)
{
  static_assert(std::is_enum<VelocityMeasPeriod>::value,
                "VelocityMeasPeriod"
                " must be an enum!");
  static const std::pair<VelocityMeasPeriod, BasicJsonType> m[] = {
    { VelocityMeasPeriod::Period_1Ms, "Period_1Ms" },
    { VelocityMeasPeriod::Period_2Ms, "Period_2Ms" },
    { VelocityMeasPeriod::Period_5Ms, "Period_5Ms" },
    { VelocityMeasPeriod::Period_10Ms, "Period_10Ms" },
    { VelocityMeasPeriod::Period_20Ms, "Period_20Ms" },
    { VelocityMeasPeriod::Period_25Ms, "Period_25Ms" },
    { VelocityMeasPeriod::Period_50Ms, "Period_50Ms" },
    { VelocityMeasPeriod::Period_100Ms, "Period_100Ms" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<VelocityMeasPeriod, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const RemoteSensorSource& e)
{
  static_assert(std::is_enum<RemoteSensorSource>::value,
                "RemoteSensorSource"
                " must be an enum!");
  static const std::pair<RemoteSensorSource, BasicJsonType> m[] = {
    { RemoteSensorSource::RemoteSensorSource_Off, "RemoteSensorSource_Off" },
    { RemoteSensorSource::RemoteSensorSource_TalonSRX_SelectedSensor,
      "RemoteSensorSource_TalonSRX_SelectedSensor" },
    { RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw,
      "RemoteSensorSource_Pigeon_Yaw" },
    { RemoteSensorSource::RemoteSensorSource_Pigeon_Pitch,
      "RemoteSensorSource_Pigeon_Pitch" },
    { RemoteSensorSource::RemoteSensorSource_Pigeon_Roll,
      "RemoteSensorSource_Pigeon_Roll" },
    { RemoteSensorSource::RemoteSensorSource_CANifier_Quadrature,
      "RemoteSensorSource_CANifier_Quadrature" },
    { RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput0,
      "RemoteSensorSource_CANifier_PWMInput0" },
    { RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput1,
      "RemoteSensorSource_CANifier_PWMInput1" },
    { RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput2,
      "RemoteSensorSource_CANifier_PWMInput2" },
    { RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput3,
      "RemoteSensorSource_CANifier_PWMInput3" },
    { RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Yaw,
      "RemoteSensorSource_GadgeteerPigeon_Yaw" },
    { RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Pitch,
      "RemoteSensorSource_GadgeteerPigeon_Pitch" },
    { RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Roll,
      "RemoteSensorSource_GadgeteerPigeon_Roll" },
    { RemoteSensorSource::RemoteSensorSource_CANCoder,
      "RemoteSensorSource_CANCoder" },
    { RemoteSensorSource::RemoteSensorSource_TalonFX_SelectedSensor,
      "RemoteSensorSource_TalonFX_SelectedSensor" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<RemoteSensorSource, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, RemoteSensorSource& e)
{
  static_assert(std::is_enum<RemoteSensorSource>::value,
                "RemoteSensorSource"
                " must be an enum!");
  static const std::pair<RemoteSensorSource, BasicJsonType> m[] = {
    { RemoteSensorSource::RemoteSensorSource_Off, "RemoteSensorSource_Off" },
    { RemoteSensorSource::RemoteSensorSource_TalonSRX_SelectedSensor,
      "RemoteSensorSource_TalonSRX_SelectedSensor" },
    { RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw,
      "RemoteSensorSource_Pigeon_Yaw" },
    { RemoteSensorSource::RemoteSensorSource_Pigeon_Pitch,
      "RemoteSensorSource_Pigeon_Pitch" },
    { RemoteSensorSource::RemoteSensorSource_Pigeon_Roll,
      "RemoteSensorSource_Pigeon_Roll" },
    { RemoteSensorSource::RemoteSensorSource_CANifier_Quadrature,
      "RemoteSensorSource_CANifier_Quadrature" },
    { RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput0,
      "RemoteSensorSource_CANifier_PWMInput0" },
    { RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput1,
      "RemoteSensorSource_CANifier_PWMInput1" },
    { RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput2,
      "RemoteSensorSource_CANifier_PWMInput2" },
    { RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput3,
      "RemoteSensorSource_CANifier_PWMInput3" },
    { RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Yaw,
      "RemoteSensorSource_GadgeteerPigeon_Yaw" },
    { RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Pitch,
      "RemoteSensorSource_GadgeteerPigeon_Pitch" },
    { RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Roll,
      "RemoteSensorSource_GadgeteerPigeon_Roll" },
    { RemoteSensorSource::RemoteSensorSource_CANCoder,
      "RemoteSensorSource_CANCoder" },
    { RemoteSensorSource::RemoteSensorSource_TalonFX_SelectedSensor,
      "RemoteSensorSource_TalonFX_SelectedSensor" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<RemoteSensorSource, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const StatusFrameEnhanced& e)
{
  static_assert(std::is_enum<StatusFrameEnhanced>::value,
                "StatusFrameEnhanced"
                " must be an enum!");
  static const std::pair<StatusFrameEnhanced, BasicJsonType> m[] = {
    { StatusFrameEnhanced::Status_1_General, "Status_1_General" },
    { StatusFrameEnhanced::Status_2_Feedback0, "Status_2_Feedback0" },
    { StatusFrameEnhanced::Status_4_AinTempVbat, "Status_4_AinTempVbat" },
    { StatusFrameEnhanced::Status_6_Misc, "Status_6_Misc" },
    { StatusFrameEnhanced::Status_7_CommStatus, "Status_7_CommStatus" },
    { StatusFrameEnhanced::Status_9_MotProfBuffer, "Status_9_MotProfBuffer" },
    { StatusFrameEnhanced::Status_10_MotionMagic, "Status_10_MotionMagic" },
    { StatusFrameEnhanced::Status_10_Targets, "Status_10_Targets" },
    { StatusFrameEnhanced::Status_12_Feedback1, "Status_12_Feedback1" },
    { StatusFrameEnhanced::Status_13_Base_PIDF0, "Status_13_Base_PIDF0" },
    { StatusFrameEnhanced::Status_14_Turn_PIDF1, "Status_14_Turn_PIDF1" },
    { StatusFrameEnhanced::Status_15_FirmareApiStatus,
      "Status_15_FirmareApiStatus" },
    { StatusFrameEnhanced::Status_17_Targets1, "Status_17_Targets1" },
    { StatusFrameEnhanced::Status_3_Quadrature, "Status_3_Quadrature" },
    { StatusFrameEnhanced::Status_8_PulseWidth, "Status_8_PulseWidth" },
    { StatusFrameEnhanced::Status_11_UartGadgeteer, "Status_11_UartGadgeteer" },
    { StatusFrameEnhanced::Status_Brushless_Current,
      "Status_Brushless_Current" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<StatusFrameEnhanced, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, StatusFrameEnhanced& e)
{
  static_assert(std::is_enum<StatusFrameEnhanced>::value,
                "StatusFrameEnhanced"
                " must be an enum!");
  static const std::pair<StatusFrameEnhanced, BasicJsonType> m[] = {
    { StatusFrameEnhanced::Status_1_General, "Status_1_General" },
    { StatusFrameEnhanced::Status_2_Feedback0, "Status_2_Feedback0" },
    { StatusFrameEnhanced::Status_4_AinTempVbat, "Status_4_AinTempVbat" },
    { StatusFrameEnhanced::Status_6_Misc, "Status_6_Misc" },
    { StatusFrameEnhanced::Status_7_CommStatus, "Status_7_CommStatus" },
    { StatusFrameEnhanced::Status_9_MotProfBuffer, "Status_9_MotProfBuffer" },
    { StatusFrameEnhanced::Status_10_MotionMagic, "Status_10_MotionMagic" },
    { StatusFrameEnhanced::Status_10_Targets, "Status_10_Targets" },
    { StatusFrameEnhanced::Status_12_Feedback1, "Status_12_Feedback1" },
    { StatusFrameEnhanced::Status_13_Base_PIDF0, "Status_13_Base_PIDF0" },
    { StatusFrameEnhanced::Status_14_Turn_PIDF1, "Status_14_Turn_PIDF1" },
    { StatusFrameEnhanced::Status_15_FirmareApiStatus,
      "Status_15_FirmareApiStatus" },
    { StatusFrameEnhanced::Status_17_Targets1, "Status_17_Targets1" },
    { StatusFrameEnhanced::Status_3_Quadrature, "Status_3_Quadrature" },
    { StatusFrameEnhanced::Status_8_PulseWidth, "Status_8_PulseWidth" },
    { StatusFrameEnhanced::Status_11_UartGadgeteer, "Status_11_UartGadgeteer" },
    { StatusFrameEnhanced::Status_Brushless_Current,
      "Status_Brushless_Current" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<StatusFrameEnhanced, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const StatusFrame& e)
{
  static_assert(std::is_enum<StatusFrame>::value,
                "StatusFrame"
                " must be an enum!");
  static const std::pair<StatusFrame, BasicJsonType> m[] = {
    { StatusFrame::Status_1_General_, "Status_1_General_" },
    { StatusFrame::Status_2_Feedback0_, "Status_2_Feedback0_" },
    { StatusFrame::Status_4_AinTempVbat_, "Status_4_AinTempVbat_" },
    { StatusFrame::Status_6_Misc_, "Status_6_Misc_" },
    { StatusFrame::Status_7_CommStatus_, "Status_7_CommStatus_" },
    { StatusFrame::Status_9_MotProfBuffer_, "Status_9_MotProfBuffer_" },
    { StatusFrame::Status_10_MotionMagic_, "Status_10_MotionMagic_" },
    { StatusFrame::Status_10_Targets_, "Status_10_Targets_" },
    { StatusFrame::Status_12_Feedback1_, "Status_12_Feedback1_" },
    { StatusFrame::Status_13_Base_PIDF0_, "Status_13_Base_PIDF0_" },
    { StatusFrame::Status_14_Turn_PIDF1_, "Status_14_Turn_PIDF1_" },
    { StatusFrame::Status_15_FirmareApiStatus_, "Status_15_FirmareApiStatus_" },
    { StatusFrame::Status_17_Targets1_, "Status_17_Targets1_" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<StatusFrame, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, StatusFrame& e)
{
  static_assert(std::is_enum<StatusFrame>::value,
                "StatusFrame"
                " must be an enum!");
  static const std::pair<StatusFrame, BasicJsonType> m[] = {
    { StatusFrame::Status_1_General_, "Status_1_General_" },
    { StatusFrame::Status_2_Feedback0_, "Status_2_Feedback0_" },
    { StatusFrame::Status_4_AinTempVbat_, "Status_4_AinTempVbat_" },
    { StatusFrame::Status_6_Misc_, "Status_6_Misc_" },
    { StatusFrame::Status_7_CommStatus_, "Status_7_CommStatus_" },
    { StatusFrame::Status_9_MotProfBuffer_, "Status_9_MotProfBuffer_" },
    { StatusFrame::Status_10_MotionMagic_, "Status_10_MotionMagic_" },
    { StatusFrame::Status_10_Targets_, "Status_10_Targets_" },
    { StatusFrame::Status_12_Feedback1_, "Status_12_Feedback1_" },
    { StatusFrame::Status_13_Base_PIDF0_, "Status_13_Base_PIDF0_" },
    { StatusFrame::Status_14_Turn_PIDF1_, "Status_14_Turn_PIDF1_" },
    { StatusFrame::Status_15_FirmareApiStatus_, "Status_15_FirmareApiStatus_" },
    { StatusFrame::Status_17_Targets1_, "Status_17_Targets1_" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<StatusFrame, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const ControlMode& e)
{
  static_assert(std::is_enum<ControlMode>::value,
                "ControlMode"
                " must be an enum!");
  static const std::pair<ControlMode, BasicJsonType> m[] = {
    { ControlMode::PercentOutput, "PercentOutput" },
    { ControlMode::Position, "Position" },
    { ControlMode::Velocity, "Velocity" },
    { ControlMode::Current, "Current" },
    { ControlMode::Follower, "Follower" },
    { ControlMode::MotionProfile, "MotionProfile" },
    { ControlMode::MotionMagic, "MotionMagic" },
    { ControlMode::MotionProfileArc, "MotionProfileArc" },
    { ControlMode::MusicTone, "MusicTone" },
    { ControlMode::Disabled, "Disabled" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<ControlMode, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, ControlMode& e)
{
  static_assert(std::is_enum<ControlMode>::value,
                "ControlMode"
                " must be an enum!");
  static const std::pair<ControlMode, BasicJsonType> m[] = {
    { ControlMode::PercentOutput, "PercentOutput" },
    { ControlMode::Position, "Position" },
    { ControlMode::Velocity, "Velocity" },
    { ControlMode::Current, "Current" },
    { ControlMode::Follower, "Follower" },
    { ControlMode::MotionProfile, "MotionProfile" },
    { ControlMode::MotionMagic, "MotionMagic" },
    { ControlMode::MotionProfileArc, "MotionProfileArc" },
    { ControlMode::MusicTone, "MusicTone" },
    { ControlMode::Disabled, "Disabled" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<ControlMode, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const TalonFXControlMode& e)
{
  static_assert(std::is_enum<TalonFXControlMode>::value,
                "TalonFXControlMode"
                " must be an enum!");
  static const std::pair<TalonFXControlMode, BasicJsonType> m[] = {
    { TalonFXControlMode::PercentOutput, "PercentOutput" },
    { TalonFXControlMode::Position, "Position" },
    { TalonFXControlMode::Velocity, "Velocity" },
    { TalonFXControlMode::Current, "Current" },
    { TalonFXControlMode::Follower, "Follower" },
    { TalonFXControlMode::MotionProfile, "MotionProfile" },
    { TalonFXControlMode::MotionMagic, "MotionMagic" },
    { TalonFXControlMode::MotionProfileArc, "MotionProfileArc" },
    { TalonFXControlMode::MusicTone, "MusicTone" },
    { TalonFXControlMode::Disabled, "Disabled" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<TalonFXControlMode, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, TalonFXControlMode& e)
{
  static_assert(std::is_enum<TalonFXControlMode>::value,
                "TalonFXControlMode"
                " must be an enum!");
  static const std::pair<TalonFXControlMode, BasicJsonType> m[] = {
    { TalonFXControlMode::PercentOutput, "PercentOutput" },
    { TalonFXControlMode::Position, "Position" },
    { TalonFXControlMode::Velocity, "Velocity" },
    { TalonFXControlMode::Current, "Current" },
    { TalonFXControlMode::Follower, "Follower" },
    { TalonFXControlMode::MotionProfile, "MotionProfile" },
    { TalonFXControlMode::MotionMagic, "MotionMagic" },
    { TalonFXControlMode::MotionProfileArc, "MotionProfileArc" },
    { TalonFXControlMode::MusicTone, "MusicTone" },
    { TalonFXControlMode::Disabled, "Disabled" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<TalonFXControlMode, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const TalonSRXControlMode& e)
{
  static_assert(std::is_enum<TalonSRXControlMode>::value,
                "TalonSRXControlMode"
                " must be an enum!");
  static const std::pair<TalonSRXControlMode, BasicJsonType> m[] = {
    { TalonSRXControlMode::PercentOutput, "PercentOutput" },
    { TalonSRXControlMode::Position, "Position" },
    { TalonSRXControlMode::Velocity, "Velocity" },
    { TalonSRXControlMode::Current, "Current" },
    { TalonSRXControlMode::Follower, "Follower" },
    { TalonSRXControlMode::MotionProfile, "MotionProfile" },
    { TalonSRXControlMode::MotionMagic, "MotionMagic" },
    { TalonSRXControlMode::MotionProfileArc, "MotionProfileArc" },
    { TalonSRXControlMode::Disabled, "Disabled" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<TalonSRXControlMode, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, TalonSRXControlMode& e)
{
  static_assert(std::is_enum<TalonSRXControlMode>::value,
                "TalonSRXControlMode"
                " must be an enum!");
  static const std::pair<TalonSRXControlMode, BasicJsonType> m[] = {
    { TalonSRXControlMode::PercentOutput, "PercentOutput" },
    { TalonSRXControlMode::Position, "Position" },
    { TalonSRXControlMode::Velocity, "Velocity" },
    { TalonSRXControlMode::Current, "Current" },
    { TalonSRXControlMode::Follower, "Follower" },
    { TalonSRXControlMode::MotionProfile, "MotionProfile" },
    { TalonSRXControlMode::MotionMagic, "MotionMagic" },
    { TalonSRXControlMode::MotionProfileArc, "MotionProfileArc" },
    { TalonSRXControlMode::Disabled, "Disabled" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<TalonSRXControlMode, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const VictorSPXControlMode& e)
{
  static_assert(std::is_enum<VictorSPXControlMode>::value,
                "VictorSPXControlMode"
                " must be an enum!");
  static const std::pair<VictorSPXControlMode, BasicJsonType> m[] = {
    { VictorSPXControlMode::PercentOutput, "PercentOutput" },
    { VictorSPXControlMode::Position, "Position" },
    { VictorSPXControlMode::Velocity, "Velocity" },
    { VictorSPXControlMode::Follower, "Follower" },
    { VictorSPXControlMode::MotionProfile, "MotionProfile" },
    { VictorSPXControlMode::MotionMagic, "MotionMagic" },
    { VictorSPXControlMode::MotionProfileArc, "MotionProfileArc" },
    { VictorSPXControlMode::Disabled, "Disabled" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<VictorSPXControlMode, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, VictorSPXControlMode& e)
{
  static_assert(std::is_enum<VictorSPXControlMode>::value,
                "VictorSPXControlMode"
                " must be an enum!");
  static const std::pair<VictorSPXControlMode, BasicJsonType> m[] = {
    { VictorSPXControlMode::PercentOutput, "PercentOutput" },
    { VictorSPXControlMode::Position, "Position" },
    { VictorSPXControlMode::Velocity, "Velocity" },
    { VictorSPXControlMode::Follower, "Follower" },
    { VictorSPXControlMode::MotionProfile, "MotionProfile" },
    { VictorSPXControlMode::MotionMagic, "MotionMagic" },
    { VictorSPXControlMode::MotionProfileArc, "MotionProfileArc" },
    { VictorSPXControlMode::Disabled, "Disabled" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<VictorSPXControlMode, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.second == j; });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const FollowerType& e)
{
  static_assert(std::is_enum<FollowerType>::value,
                "FollowerType"
                " must be an enum!");
  static const std::pair<FollowerType, BasicJsonType> m[] = {
    { FollowerType::FollowerType_PercentOutput, "FollowerType_PercentOutput" },
    { FollowerType::FollowerType_AuxOutput1, "FollowerType_AuxOutput1" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<FollowerType, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, FollowerType& e)
{
  static_assert(std::is_enum<FollowerType>::value,
                "FollowerType"
                " must be an enum!");
  static const std::pair<FollowerType, BasicJsonType> m[] = {
    { FollowerType::FollowerType_PercentOutput, "FollowerType_PercentOutput" },
    { FollowerType::FollowerType_AuxOutput1, "FollowerType_AuxOutput1" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<FollowerType, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const FeedbackDevice& e)
{
  static_assert(std::is_enum<FeedbackDevice>::value,
                "FeedbackDevice"
                " must be an enum!");
  static const std::pair<FeedbackDevice, BasicJsonType> m[] = {
    { FeedbackDevice::QuadEncoder, "QuadEncoder" },
    { FeedbackDevice::IntegratedSensor, "IntegratedSensor" },
    { FeedbackDevice::Analog, "Analog" },
    { FeedbackDevice::Tachometer, "Tachometer" },
    { FeedbackDevice::PulseWidthEncodedPosition, "PulseWidthEncodedPosition" },
    { FeedbackDevice::SensorSum, "SensorSum" },
    { FeedbackDevice::SensorDifference, "SensorDifference" },
    { FeedbackDevice::RemoteSensor0, "RemoteSensor0" },
    { FeedbackDevice::RemoteSensor1, "RemoteSensor1" },
    { FeedbackDevice::None, "None" },
    { FeedbackDevice::SoftwareEmulatedSensor, "SoftwareEmulatedSensor" },
    { FeedbackDevice::CTRE_MagEncoder_Absolute, "CTRE_MagEncoder_Absolute" },
    { FeedbackDevice::CTRE_MagEncoder_Relative, "CTRE_MagEncoder_Relative" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<FeedbackDevice, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, FeedbackDevice& e)
{
  static_assert(std::is_enum<FeedbackDevice>::value,
                "FeedbackDevice"
                " must be an enum!");
  static const std::pair<FeedbackDevice, BasicJsonType> m[] = {
    { FeedbackDevice::QuadEncoder, "QuadEncoder" },
    { FeedbackDevice::IntegratedSensor, "IntegratedSensor" },
    { FeedbackDevice::Analog, "Analog" },
    { FeedbackDevice::Tachometer, "Tachometer" },
    { FeedbackDevice::PulseWidthEncodedPosition, "PulseWidthEncodedPosition" },
    { FeedbackDevice::SensorSum, "SensorSum" },
    { FeedbackDevice::SensorDifference, "SensorDifference" },
    { FeedbackDevice::RemoteSensor0, "RemoteSensor0" },
    { FeedbackDevice::RemoteSensor1, "RemoteSensor1" },
    { FeedbackDevice::None, "None" },
    { FeedbackDevice::SoftwareEmulatedSensor, "SoftwareEmulatedSensor" },
    { FeedbackDevice::CTRE_MagEncoder_Absolute, "CTRE_MagEncoder_Absolute" },
    { FeedbackDevice::CTRE_MagEncoder_Relative, "CTRE_MagEncoder_Relative" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<FeedbackDevice, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const TalonSRXFeedbackDevice& e)
{
  static_assert(std::is_enum<TalonSRXFeedbackDevice>::value,
                "TalonSRXFeedbackDevice"
                " must be an enum!");
  static const std::pair<TalonSRXFeedbackDevice, BasicJsonType> m[] = {
    { TalonSRXFeedbackDevice::QuadEncoder, "QuadEncoder" },
    { TalonSRXFeedbackDevice::Analog, "Analog" },
    { TalonSRXFeedbackDevice::Tachometer, "Tachometer" },
    { TalonSRXFeedbackDevice::PulseWidthEncodedPosition,
      "PulseWidthEncodedPosition" },
    { TalonSRXFeedbackDevice::SensorSum, "SensorSum" },
    { TalonSRXFeedbackDevice::SensorDifference, "SensorDifference" },
    { TalonSRXFeedbackDevice::RemoteSensor0, "RemoteSensor0" },
    { TalonSRXFeedbackDevice::RemoteSensor1, "RemoteSensor1" },
    { TalonSRXFeedbackDevice::None, "None" },
    { TalonSRXFeedbackDevice::SoftwareEmulatedSensor,
      "SoftwareEmulatedSensor" },
    { TalonSRXFeedbackDevice::CTRE_MagEncoder_Absolute,
      "CTRE_MagEncoder_Absolute" },
    { TalonSRXFeedbackDevice::CTRE_MagEncoder_Relative,
      "CTRE_MagEncoder_Relative" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<TalonSRXFeedbackDevice, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.first == e; });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, TalonSRXFeedbackDevice& e)
{
  static_assert(std::is_enum<TalonSRXFeedbackDevice>::value,
                "TalonSRXFeedbackDevice"
                " must be an enum!");
  static const std::pair<TalonSRXFeedbackDevice, BasicJsonType> m[] = {
    { TalonSRXFeedbackDevice::QuadEncoder, "QuadEncoder" },
    { TalonSRXFeedbackDevice::Analog, "Analog" },
    { TalonSRXFeedbackDevice::Tachometer, "Tachometer" },
    { TalonSRXFeedbackDevice::PulseWidthEncodedPosition,
      "PulseWidthEncodedPosition" },
    { TalonSRXFeedbackDevice::SensorSum, "SensorSum" },
    { TalonSRXFeedbackDevice::SensorDifference, "SensorDifference" },
    { TalonSRXFeedbackDevice::RemoteSensor0, "RemoteSensor0" },
    { TalonSRXFeedbackDevice::RemoteSensor1, "RemoteSensor1" },
    { TalonSRXFeedbackDevice::None, "None" },
    { TalonSRXFeedbackDevice::SoftwareEmulatedSensor,
      "SoftwareEmulatedSensor" },
    { TalonSRXFeedbackDevice::CTRE_MagEncoder_Absolute,
      "CTRE_MagEncoder_Absolute" },
    { TalonSRXFeedbackDevice::CTRE_MagEncoder_Relative,
      "CTRE_MagEncoder_Relative" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<TalonSRXFeedbackDevice, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.second == j; });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const TalonFXFeedbackDevice& e)
{
  static_assert(std::is_enum<TalonFXFeedbackDevice>::value,
                "TalonFXFeedbackDevice"
                " must be an enum!");
  static const std::pair<TalonFXFeedbackDevice, BasicJsonType> m[] = {
    { TalonFXFeedbackDevice::IntegratedSensor, "IntegratedSensor" },
    { TalonFXFeedbackDevice::SensorSum, "SensorSum" },
    { TalonFXFeedbackDevice::SensorDifference, "SensorDifference" },
    { TalonFXFeedbackDevice::RemoteSensor0, "RemoteSensor0" },
    { TalonFXFeedbackDevice::RemoteSensor1, "RemoteSensor1" },
    { TalonFXFeedbackDevice::None, "None" },
    { TalonFXFeedbackDevice::SoftwareEmulatedSensor, "SoftwareEmulatedSensor" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<TalonFXFeedbackDevice, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.first == e; });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, TalonFXFeedbackDevice& e)
{
  static_assert(std::is_enum<TalonFXFeedbackDevice>::value,
                "TalonFXFeedbackDevice"
                " must be an enum!");
  static const std::pair<TalonFXFeedbackDevice, BasicJsonType> m[] = {
    { TalonFXFeedbackDevice::IntegratedSensor, "IntegratedSensor" },
    { TalonFXFeedbackDevice::SensorSum, "SensorSum" },
    { TalonFXFeedbackDevice::SensorDifference, "SensorDifference" },
    { TalonFXFeedbackDevice::RemoteSensor0, "RemoteSensor0" },
    { TalonFXFeedbackDevice::RemoteSensor1, "RemoteSensor1" },
    { TalonFXFeedbackDevice::None, "None" },
    { TalonFXFeedbackDevice::SoftwareEmulatedSensor, "SoftwareEmulatedSensor" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<TalonFXFeedbackDevice, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.second == j; });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const RemoteFeedbackDevice& e)
{
  static_assert(std::is_enum<RemoteFeedbackDevice>::value,
                "RemoteFeedbackDevice"
                " must be an enum!");
  static const std::pair<RemoteFeedbackDevice, BasicJsonType> m[] = {
    { RemoteFeedbackDevice::RemoteFeedbackDevice_FactoryDefaultOff,
      "RemoteFeedbackDevice_FactoryDefaultOff" },
    { RemoteFeedbackDevice::FactoryDefaultOff, "FactoryDefaultOff" },
    { RemoteFeedbackDevice::RemoteFeedbackDevice_SensorSum,
      "RemoteFeedbackDevice_SensorSum" },
    { RemoteFeedbackDevice::SensorSum, "SensorSum" },
    { RemoteFeedbackDevice::RemoteFeedbackDevice_SensorDifference,
      "RemoteFeedbackDevice_SensorDifference" },
    { RemoteFeedbackDevice::SensorDifference, "SensorDifference" },
    { RemoteFeedbackDevice::RemoteFeedbackDevice_RemoteSensor0,
      "RemoteFeedbackDevice_RemoteSensor0" },
    { RemoteFeedbackDevice::RemoteSensor0, "RemoteSensor0" },
    { RemoteFeedbackDevice::RemoteFeedbackDevice_RemoteSensor1,
      "RemoteFeedbackDevice_RemoteSensor1" },
    { RemoteFeedbackDevice::RemoteSensor1, "RemoteSensor1" },
    { RemoteFeedbackDevice::RemoteFeedbackDevice_None,
      "RemoteFeedbackDevice_None" },
    { RemoteFeedbackDevice::None, "None" },
    { RemoteFeedbackDevice::RemoteFeedbackDevice_SoftwareEmulatedSensor,
      "RemoteFeedbackDevice_SoftwareEmulatedSensor" },
    { RemoteFeedbackDevice::SoftwareEmulatedSensor, "SoftwareEmulatedSensor" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<RemoteFeedbackDevice, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, RemoteFeedbackDevice& e)
{
  static_assert(std::is_enum<RemoteFeedbackDevice>::value,
                "RemoteFeedbackDevice"
                " must be an enum!");
  static const std::pair<RemoteFeedbackDevice, BasicJsonType> m[] = {
    { RemoteFeedbackDevice::RemoteFeedbackDevice_FactoryDefaultOff,
      "RemoteFeedbackDevice_FactoryDefaultOff" },
    { RemoteFeedbackDevice::FactoryDefaultOff, "FactoryDefaultOff" },
    { RemoteFeedbackDevice::RemoteFeedbackDevice_SensorSum,
      "RemoteFeedbackDevice_SensorSum" },
    { RemoteFeedbackDevice::SensorSum, "SensorSum" },
    { RemoteFeedbackDevice::RemoteFeedbackDevice_SensorDifference,
      "RemoteFeedbackDevice_SensorDifference" },
    { RemoteFeedbackDevice::SensorDifference, "SensorDifference" },
    { RemoteFeedbackDevice::RemoteFeedbackDevice_RemoteSensor0,
      "RemoteFeedbackDevice_RemoteSensor0" },
    { RemoteFeedbackDevice::RemoteSensor0, "RemoteSensor0" },
    { RemoteFeedbackDevice::RemoteFeedbackDevice_RemoteSensor1,
      "RemoteFeedbackDevice_RemoteSensor1" },
    { RemoteFeedbackDevice::RemoteSensor1, "RemoteSensor1" },
    { RemoteFeedbackDevice::RemoteFeedbackDevice_None,
      "RemoteFeedbackDevice_None" },
    { RemoteFeedbackDevice::None, "None" },
    { RemoteFeedbackDevice::RemoteFeedbackDevice_SoftwareEmulatedSensor,
      "RemoteFeedbackDevice_SoftwareEmulatedSensor" },
    { RemoteFeedbackDevice::SoftwareEmulatedSensor, "SoftwareEmulatedSensor" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<RemoteFeedbackDevice, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.second == j; });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const SensorTerm& e)
{
  static_assert(std::is_enum<SensorTerm>::value,
                "SensorTerm"
                " must be an enum!");
  static const std::pair<SensorTerm, BasicJsonType> m[] = {
    { SensorTerm::SensorTerm_Sum0, "SensorTerm_Sum0" },
    { SensorTerm::SensorTerm_Sum1, "SensorTerm_Sum1" },
    { SensorTerm::SensorTerm_Diff0, "SensorTerm_Diff0" },
    { SensorTerm::SensorTerm_Diff1, "SensorTerm_Diff1" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<SensorTerm, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, SensorTerm& e)
{
  static_assert(std::is_enum<SensorTerm>::value,
                "SensorTerm"
                " must be an enum!");
  static const std::pair<SensorTerm, BasicJsonType> m[] = {
    { SensorTerm::SensorTerm_Sum0, "SensorTerm_Sum0" },
    { SensorTerm::SensorTerm_Sum1, "SensorTerm_Sum1" },
    { SensorTerm::SensorTerm_Diff0, "SensorTerm_Diff0" },
    { SensorTerm::SensorTerm_Diff1, "SensorTerm_Diff1" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<SensorTerm, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const InvertType& e)
{
  static_assert(std::is_enum<InvertType>::value,
                "InvertType"
                " must be an enum!");
  static const std::pair<InvertType, BasicJsonType> m[] = {
    { InvertType::None, "None" },
    { InvertType::InvertMotorOutput, "InvertMotorOutput" },
    { InvertType::FollowMaster, "FollowMaster" },
    { InvertType::OpposeMaster, "OpposeMaster" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<InvertType, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, InvertType& e)
{
  static_assert(std::is_enum<InvertType>::value,
                "InvertType"
                " must be an enum!");
  static const std::pair<InvertType, BasicJsonType> m[] = {
    { InvertType::None, "None" },
    { InvertType::InvertMotorOutput, "InvertMotorOutput" },
    { InvertType::FollowMaster, "FollowMaster" },
    { InvertType::OpposeMaster, "OpposeMaster" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<InvertType, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const TalonFXInvertType& e)
{
  static_assert(std::is_enum<TalonFXInvertType>::value,
                "TalonFXInvertType"
                " must be an enum!");
  static const std::pair<TalonFXInvertType, BasicJsonType> m[] = {
    { TalonFXInvertType::CounterClockwise, "CounterClockwise" },
    { TalonFXInvertType::Clockwise, "Clockwise" },
    { TalonFXInvertType::FollowMaster, "FollowMaster" },
    { TalonFXInvertType::OpposeMaster, "OpposeMaster" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<TalonFXInvertType, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, TalonFXInvertType& e)
{
  static_assert(std::is_enum<TalonFXInvertType>::value,
                "TalonFXInvertType"
                " must be an enum!");
  static const std::pair<TalonFXInvertType, BasicJsonType> m[] = {
    { TalonFXInvertType::CounterClockwise, "CounterClockwise" },
    { TalonFXInvertType::Clockwise, "Clockwise" },
    { TalonFXInvertType::FollowMaster, "FollowMaster" },
    { TalonFXInvertType::OpposeMaster, "OpposeMaster" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<TalonFXInvertType, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const DemandType& e)
{
  static_assert(std::is_enum<DemandType>::value,
                "DemandType"
                " must be an enum!");
  static const std::pair<DemandType, BasicJsonType> m[] = {
    { DemandType::DemandType_Neutral, "DemandType_Neutral" },
    { DemandType::DemandType_AuxPID, "DemandType_AuxPID" },
    { DemandType::DemandType_ArbitraryFeedForward,
      "DemandType_ArbitraryFeedForward" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<DemandType, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, DemandType& e)
{
  static_assert(std::is_enum<DemandType>::value,
                "DemandType"
                " must be an enum!");
  static const std::pair<DemandType, BasicJsonType> m[] = {
    { DemandType::DemandType_Neutral, "DemandType_Neutral" },
    { DemandType::DemandType_AuxPID, "DemandType_AuxPID" },
    { DemandType::DemandType_ArbitraryFeedForward,
      "DemandType_ArbitraryFeedForward" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<DemandType, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const MotorCommutation& e)
{
  static_assert(std::is_enum<MotorCommutation>::value,
                "MotorCommutation"
                " must be an enum!");
  static const std::pair<MotorCommutation, BasicJsonType> m[] = {
    { MotorCommutation::Trapezoidal, "Trapezoidal" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<MotorCommutation, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, MotorCommutation& e)
{
  static_assert(std::is_enum<MotorCommutation>::value,
                "MotorCommutation"
                " must be an enum!");
  static const std::pair<MotorCommutation, BasicJsonType> m[] = {
    { MotorCommutation::Trapezoidal, "Trapezoidal" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<MotorCommutation, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const LimitSwitchSource& e)
{
  static_assert(std::is_enum<LimitSwitchSource>::value,
                "LimitSwitchSource"
                " must be an enum!");
  static const std::pair<LimitSwitchSource, BasicJsonType> m[] = {
    { LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
      "LimitSwitchSource_FeedbackConnector" },
    { LimitSwitchSource::LimitSwitchSource_RemoteTalonSRX,
      "LimitSwitchSource_RemoteTalonSRX" },
    { LimitSwitchSource::LimitSwitchSource_RemoteCANifier,
      "LimitSwitchSource_RemoteCANifier" },
    { LimitSwitchSource::LimitSwitchSource_Deactivated,
      "LimitSwitchSource_Deactivated" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<LimitSwitchSource, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, LimitSwitchSource& e)
{
  static_assert(std::is_enum<LimitSwitchSource>::value,
                "LimitSwitchSource"
                " must be an enum!");
  static const std::pair<LimitSwitchSource, BasicJsonType> m[] = {
    { LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
      "LimitSwitchSource_FeedbackConnector" },
    { LimitSwitchSource::LimitSwitchSource_RemoteTalonSRX,
      "LimitSwitchSource_RemoteTalonSRX" },
    { LimitSwitchSource::LimitSwitchSource_RemoteCANifier,
      "LimitSwitchSource_RemoteCANifier" },
    { LimitSwitchSource::LimitSwitchSource_Deactivated,
      "LimitSwitchSource_Deactivated" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<LimitSwitchSource, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const RemoteLimitSwitchSource& e)
{
  static_assert(std::is_enum<RemoteLimitSwitchSource>::value,
                "RemoteLimitSwitchSource"
                " must be an enum!");
  static const std::pair<RemoteLimitSwitchSource, BasicJsonType> m[] = {
    { RemoteLimitSwitchSource::RemoteLimitSwitchSource_RemoteTalonSRX,
      "RemoteLimitSwitchSource_RemoteTalonSRX" },
    { RemoteLimitSwitchSource::RemoteLimitSwitchSource_RemoteCANifier,
      "RemoteLimitSwitchSource_RemoteCANifier" },
    { RemoteLimitSwitchSource::RemoteLimitSwitchSource_Deactivated,
      "RemoteLimitSwitchSource_Deactivated" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<RemoteLimitSwitchSource, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.first == e; });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, RemoteLimitSwitchSource& e)
{
  static_assert(std::is_enum<RemoteLimitSwitchSource>::value,
                "RemoteLimitSwitchSource"
                " must be an enum!");
  static const std::pair<RemoteLimitSwitchSource, BasicJsonType> m[] = {
    { RemoteLimitSwitchSource::RemoteLimitSwitchSource_RemoteTalonSRX,
      "RemoteLimitSwitchSource_RemoteTalonSRX" },
    { RemoteLimitSwitchSource::RemoteLimitSwitchSource_RemoteCANifier,
      "RemoteLimitSwitchSource_RemoteCANifier" },
    { RemoteLimitSwitchSource::RemoteLimitSwitchSource_Deactivated,
      "RemoteLimitSwitchSource_Deactivated" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<RemoteLimitSwitchSource, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.second == j; });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const LimitSwitchNormal& e)
{
  static_assert(std::is_enum<LimitSwitchNormal>::value,
                "LimitSwitchNormal"
                " must be an enum!");
  static const std::pair<LimitSwitchNormal, BasicJsonType> m[] = {
    { LimitSwitchNormal::LimitSwitchNormal_NormallyOpen,
      "LimitSwitchNormal_NormallyOpen" },
    { LimitSwitchNormal::LimitSwitchNormal_NormallyClosed,
      "LimitSwitchNormal_NormallyClosed" },
    { LimitSwitchNormal::LimitSwitchNormal_Disabled,
      "LimitSwitchNormal_Disabled" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<LimitSwitchNormal, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, LimitSwitchNormal& e)
{
  static_assert(std::is_enum<LimitSwitchNormal>::value,
                "LimitSwitchNormal"
                " must be an enum!");
  static const std::pair<LimitSwitchNormal, BasicJsonType> m[] = {
    { LimitSwitchNormal::LimitSwitchNormal_NormallyOpen,
      "LimitSwitchNormal_NormallyOpen" },
    { LimitSwitchNormal::LimitSwitchNormal_NormallyClosed,
      "LimitSwitchNormal_NormallyClosed" },
    { LimitSwitchNormal::LimitSwitchNormal_Disabled,
      "LimitSwitchNormal_Disabled" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<LimitSwitchNormal, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const ControlFrame& e)
{
  static_assert(std::is_enum<ControlFrame>::value,
                "ControlFrame"
                " must be an enum!");
  static const std::pair<ControlFrame, BasicJsonType> m[] = {
    { ControlFrame::Control_3_General, "Control_3_General" },
    { ControlFrame::Control_4_Advanced, "Control_4_Advanced" },
    { ControlFrame::Control_6_MotProfAddTrajPoint,
      "Control_6_MotProfAddTrajPoint" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<ControlFrame, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, ControlFrame& e)
{
  static_assert(std::is_enum<ControlFrame>::value,
                "ControlFrame"
                " must be an enum!");
  static const std::pair<ControlFrame, BasicJsonType> m[] = {
    { ControlFrame::Control_3_General, "Control_3_General" },
    { ControlFrame::Control_4_Advanced, "Control_4_Advanced" },
    { ControlFrame::Control_6_MotProfAddTrajPoint,
      "Control_6_MotProfAddTrajPoint" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<ControlFrame, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const ControlFrameEnhanced& e)
{
  static_assert(std::is_enum<ControlFrameEnhanced>::value,
                "ControlFrameEnhanced"
                " must be an enum!");
  static const std::pair<ControlFrameEnhanced, BasicJsonType> m[] = {
    { ControlFrameEnhanced::Control_3_General_, "Control_3_General" },
    { ControlFrameEnhanced::Control_4_Advanced_, "Control_4_Advanced" },
    { ControlFrameEnhanced::Control_5_FeedbackOutputOverride_,
      "Control_5_FeedbackOutputOverride_" },
    { ControlFrameEnhanced::Control_6_MotProfAddTrajPoint_,
      "Control_6_MotProfAddTrajPoint" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<ControlFrameEnhanced, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, ControlFrameEnhanced& e)
{
  static_assert(std::is_enum<ControlFrameEnhanced>::value,
                "ControlFrameEnhanced"
                " must be an enum!");
  static const std::pair<ControlFrameEnhanced, BasicJsonType> m[] = {
    { ControlFrameEnhanced::Control_3_General_, "Control_3_General" },
    { ControlFrameEnhanced::Control_4_Advanced_, "Control_4_Advanced" },
    { ControlFrameEnhanced::Control_5_FeedbackOutputOverride_,
      "Control_5_FeedbackOutputOverride_" },
    { ControlFrameEnhanced::Control_6_MotProfAddTrajPoint_,
      "Control_6_MotProfAddTrajPoint" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<ControlFrameEnhanced, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.second == j; });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

}
namespace motion {
template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const SetValueMotionProfile& e)
{
  static_assert(std::is_enum<SetValueMotionProfile>::value,
                "SetValueMotionProfile"
                " must be an enum!");
  static const std::pair<SetValueMotionProfile, BasicJsonType> m[] = {
    { SetValueMotionProfile::Disable, "Disable" },
    { SetValueMotionProfile::Enable, "Enable" },
    { SetValueMotionProfile::Hold, "Hold" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<SetValueMotionProfile, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.first == e; });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, SetValueMotionProfile& e)
{
  static_assert(std::is_enum<SetValueMotionProfile>::value,
                "SetValueMotionProfile"
                " must be an enum!");
  static const std::pair<SetValueMotionProfile, BasicJsonType> m[] = {
    { SetValueMotionProfile::Disable, "Disable" },
    { SetValueMotionProfile::Enable, "Enable" },
    { SetValueMotionProfile::Hold, "Hold" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<SetValueMotionProfile, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.second == j; });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

}
namespace sensors {
template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const CANCoderStatusFrame& e)
{
  static_assert(std::is_enum<CANCoderStatusFrame>::value,
                "CANCoderStatusFrame"
                " must be an enum!");
  static const std::pair<CANCoderStatusFrame, BasicJsonType> m[] = {
    { CANCoderStatusFrame::CANCoderStatusFrame_SensorData,
      "CANCoderStatusFrame_SensorData" },
    { CANCoderStatusFrame::CANCoderStatusFrame_VbatAndFaults,
      "CANCoderStatusFrame_VbatAndFaults" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<CANCoderStatusFrame, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, CANCoderStatusFrame& e)
{
  static_assert(std::is_enum<CANCoderStatusFrame>::value,
                "CANCoderStatusFrame"
                " must be an enum!");
  static const std::pair<CANCoderStatusFrame, BasicJsonType> m[] = {
    { CANCoderStatusFrame::CANCoderStatusFrame_SensorData,
      "CANCoderStatusFrame_SensorData" },
    { CANCoderStatusFrame::CANCoderStatusFrame_VbatAndFaults,
      "CANCoderStatusFrame_VbatAndFaults" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<CANCoderStatusFrame, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const SensorTimeBase& e)
{
  static_assert(std::is_enum<SensorTimeBase>::value,
                "SensorTimeBase"
                " must be an enum!");
  static const std::pair<SensorTimeBase, BasicJsonType> m[] = {
    { SensorTimeBase::Per100Ms_Legacy, "Per100Ms_Legacy" },
    { SensorTimeBase::PerSecond, "PerSecond" },
    { SensorTimeBase::PerMinute, "PerMinute" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<SensorTimeBase, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, SensorTimeBase& e)
{
  static_assert(std::is_enum<SensorTimeBase>::value,
                "SensorTimeBase"
                " must be an enum!");
  static const std::pair<SensorTimeBase, BasicJsonType> m[] = {
    { SensorTimeBase::Per100Ms_Legacy, "Per100Ms_Legacy" },
    { SensorTimeBase::PerSecond, "PerSecond" },
    { SensorTimeBase::PerMinute, "PerMinute" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<SensorTimeBase, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const PigeonIMU_ControlFrame& e)
{
  static_assert(std::is_enum<PigeonIMU_ControlFrame>::value,
                "PigeonIMU_ControlFrame"
                " must be an enum!");
  static const std::pair<PigeonIMU_ControlFrame, BasicJsonType> m[] = {
    { PigeonIMU_ControlFrame::PigeonIMU_CondStatus_Control_1,
      "PigeonIMU_CondStatus_Control_1" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<PigeonIMU_ControlFrame, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.first == e; });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, PigeonIMU_ControlFrame& e)
{
  static_assert(std::is_enum<PigeonIMU_ControlFrame>::value,
                "PigeonIMU_ControlFrame"
                " must be an enum!");
  static const std::pair<PigeonIMU_ControlFrame, BasicJsonType> m[] = {
    { PigeonIMU_ControlFrame::PigeonIMU_CondStatus_Control_1,
      "PigeonIMU_CondStatus_Control_1" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<PigeonIMU_ControlFrame, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.second == j; });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const MagnetFieldStrength& e)
{
  static_assert(std::is_enum<MagnetFieldStrength>::value,
                "MagnetFieldStrength"
                " must be an enum!");
  static const std::pair<MagnetFieldStrength, BasicJsonType> m[] = {
    { MagnetFieldStrength::Invalid_Unknown, "Invalid_Unknown" },
    { MagnetFieldStrength::BadRange_RedLED, "BadRange_RedLED" },
    { MagnetFieldStrength::Adequate_OrangeLED, "Adequate_OrangeLED" },
    { MagnetFieldStrength::Good_GreenLED, "Good_GreenLED" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<MagnetFieldStrength, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, MagnetFieldStrength& e)
{
  static_assert(std::is_enum<MagnetFieldStrength>::value,
                "MagnetFieldStrength"
                " must be an enum!");
  static const std::pair<MagnetFieldStrength, BasicJsonType> m[] = {
    { MagnetFieldStrength::Invalid_Unknown, "Invalid_Unknown" },
    { MagnetFieldStrength::BadRange_RedLED, "BadRange_RedLED" },
    { MagnetFieldStrength::Adequate_OrangeLED, "Adequate_OrangeLED" },
    { MagnetFieldStrength::Good_GreenLED, "Good_GreenLED" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<MagnetFieldStrength, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const AbsoluteSensorRange& e)
{
  static_assert(std::is_enum<AbsoluteSensorRange>::value,
                "AbsoluteSensorRange"
                " must be an enum!");
  static const std::pair<AbsoluteSensorRange, BasicJsonType> m[] = {
    { AbsoluteSensorRange::Unsigned_0_to_360, "Unsigned_0_to_360" },
    { AbsoluteSensorRange::Signed_PlusMinus180, "Signed_PlusMinus180" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<AbsoluteSensorRange, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.first == e;
    });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, AbsoluteSensorRange& e)
{
  static_assert(std::is_enum<AbsoluteSensorRange>::value,
                "AbsoluteSensorRange"
                " must be an enum!");
  static const std::pair<AbsoluteSensorRange, BasicJsonType> m[] = {
    { AbsoluteSensorRange::Unsigned_0_to_360, "Unsigned_0_to_360" },
    { AbsoluteSensorRange::Signed_PlusMinus180, "Signed_PlusMinus180" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<AbsoluteSensorRange, BasicJsonType>& ej_pair) -> bool {
      return ej_pair.second == j;
    });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const PigeonIMU_StatusFrame& e)
{
  static_assert(std::is_enum<PigeonIMU_StatusFrame>::value,
                "PigeonIMU_StatusFrame"
                " must be an enum!");
  static const std::pair<PigeonIMU_StatusFrame, BasicJsonType> m[] = {
    { PigeonIMU_StatusFrame::PigeonIMU_CondStatus_1_General,
      "PigeonIMU_CondStatus_1_General" },
    { PigeonIMU_StatusFrame::PigeonIMU_CondStatus_9_SixDeg_YPR,
      "PigeonIMU_CondStatus_9_SixDeg_YPR" },
    { PigeonIMU_StatusFrame::PigeonIMU_CondStatus_6_SensorFusion,
      "PigeonIMU_CondStatus_6_SensorFusion" },
    { PigeonIMU_StatusFrame::PigeonIMU_CondStatus_11_GyroAccum,
      "PigeonIMU_CondStatus_11_GyroAccum" },
    { PigeonIMU_StatusFrame::PigeonIMU_CondStatus_2_GeneralCompass,
      "PigeonIMU_CondStatus_2_GeneralCompass" },
    { PigeonIMU_StatusFrame::PigeonIMU_CondStatus_3_GeneralAccel,
      "PigeonIMU_CondStatus_3_GeneralAccel" },
    { PigeonIMU_StatusFrame::PigeonIMU_CondStatus_10_SixDeg_Quat,
      "PigeonIMU_CondStatus_10_SixDeg_Quat" },
    { PigeonIMU_StatusFrame::PigeonIMU_RawStatus_4_Mag,
      "PigeonIMU_RawStatus_4_Mag" },
    { PigeonIMU_StatusFrame::PigeonIMU_BiasedStatus_2_Gyro,
      "PigeonIMU_BiasedStatus_2_Gyro" },
    { PigeonIMU_StatusFrame::PigeonIMU_BiasedStatus_4_Mag,
      "PigeonIMU_BiasedStatus_4_Mag" },
    { PigeonIMU_StatusFrame::PigeonIMU_BiasedStatus_6_Accel,
      "PigeonIMU_BiasedStatus_6_Accel" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<PigeonIMU_StatusFrame, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.first == e; });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, PigeonIMU_StatusFrame& e)
{
  static_assert(std::is_enum<PigeonIMU_StatusFrame>::value,
                "PigeonIMU_StatusFrame"
                " must be an enum!");
  static const std::pair<PigeonIMU_StatusFrame, BasicJsonType> m[] = {
    { PigeonIMU_StatusFrame::PigeonIMU_CondStatus_1_General,
      "PigeonIMU_CondStatus_1_General" },
    { PigeonIMU_StatusFrame::PigeonIMU_CondStatus_9_SixDeg_YPR,
      "PigeonIMU_CondStatus_9_SixDeg_YPR" },
    { PigeonIMU_StatusFrame::PigeonIMU_CondStatus_6_SensorFusion,
      "PigeonIMU_CondStatus_6_SensorFusion" },
    { PigeonIMU_StatusFrame::PigeonIMU_CondStatus_11_GyroAccum,
      "PigeonIMU_CondStatus_11_GyroAccum" },
    { PigeonIMU_StatusFrame::PigeonIMU_CondStatus_2_GeneralCompass,
      "PigeonIMU_CondStatus_2_GeneralCompass" },
    { PigeonIMU_StatusFrame::PigeonIMU_CondStatus_3_GeneralAccel,
      "PigeonIMU_CondStatus_3_GeneralAccel" },
    { PigeonIMU_StatusFrame::PigeonIMU_CondStatus_10_SixDeg_Quat,
      "PigeonIMU_CondStatus_10_SixDeg_Quat" },
    { PigeonIMU_StatusFrame::PigeonIMU_RawStatus_4_Mag,
      "PigeonIMU_RawStatus_4_Mag" },
    { PigeonIMU_StatusFrame::PigeonIMU_BiasedStatus_2_Gyro,
      "PigeonIMU_BiasedStatus_2_Gyro" },
    { PigeonIMU_StatusFrame::PigeonIMU_BiasedStatus_4_Mag,
      "PigeonIMU_BiasedStatus_4_Mag" },
    { PigeonIMU_StatusFrame::PigeonIMU_BiasedStatus_6_Accel,
      "PigeonIMU_BiasedStatus_6_Accel" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<PigeonIMU_StatusFrame, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.second == j; });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const SensorVelocityMeasPeriod& e)
{
  static_assert(std::is_enum<SensorVelocityMeasPeriod>::value,
                "SensorVelocityMeasPeriod"
                " must be an enum!");
  static const std::pair<SensorVelocityMeasPeriod, BasicJsonType> m[] = {
    { SensorVelocityMeasPeriod::Period_1Ms, "Period_1Ms" },
    { SensorVelocityMeasPeriod::Period_2Ms, "Period_2Ms" },
    { SensorVelocityMeasPeriod::Period_5Ms, "Period_5Ms" },
    { SensorVelocityMeasPeriod::Period_10Ms, "Period_10Ms" },
    { SensorVelocityMeasPeriod::Period_20Ms, "Period_20Ms" },
    { SensorVelocityMeasPeriod::Period_25Ms, "Period_25Ms" },
    { SensorVelocityMeasPeriod::Period_50Ms, "Period_50Ms" },
    { SensorVelocityMeasPeriod::Period_100Ms, "Period_100Ms" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<SensorVelocityMeasPeriod, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.first == e; });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, SensorVelocityMeasPeriod& e)
{
  static_assert(std::is_enum<SensorVelocityMeasPeriod>::value,
                "SensorVelocityMeasPeriod"
                " must be an enum!");
  static const std::pair<SensorVelocityMeasPeriod, BasicJsonType> m[] = {
    { SensorVelocityMeasPeriod::Period_1Ms, "Period_1Ms" },
    { SensorVelocityMeasPeriod::Period_2Ms, "Period_2Ms" },
    { SensorVelocityMeasPeriod::Period_5Ms, "Period_5Ms" },
    { SensorVelocityMeasPeriod::Period_10Ms, "Period_10Ms" },
    { SensorVelocityMeasPeriod::Period_20Ms, "Period_20Ms" },
    { SensorVelocityMeasPeriod::Period_25Ms, "Period_25Ms" },
    { SensorVelocityMeasPeriod::Period_50Ms, "Period_50Ms" },
    { SensorVelocityMeasPeriod::Period_100Ms, "Period_100Ms" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<SensorVelocityMeasPeriod, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.second == j; });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const SensorInitializationStrategy& e)
{
  static_assert(std::is_enum<SensorInitializationStrategy>::value,
                "SensorInitializationStrategy"
                " must be an enum!");
  static const std::pair<SensorInitializationStrategy, BasicJsonType> m[] = {
    { SensorInitializationStrategy::BootToZero, "BootToZero" },
    { SensorInitializationStrategy::BootToAbsolutePosition,
      "BootToAbsolutePosition" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<SensorInitializationStrategy, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.first == e; });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, SensorInitializationStrategy& e)
{
  static_assert(std::is_enum<SensorInitializationStrategy>::value,
                "SensorInitializationStrategy"
                " must be an enum!");
  static const std::pair<SensorInitializationStrategy, BasicJsonType> m[] = {
    { SensorInitializationStrategy::BootToZero, "BootToZero" },
    { SensorInitializationStrategy::BootToAbsolutePosition,
      "BootToAbsolutePosition" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<SensorInitializationStrategy, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.second == j; });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const PigeonIMU::CalibrationMode& e)
{
  static_assert(std::is_enum<PigeonIMU::CalibrationMode>::value,
                "PigeonIMU::CalibrationMode"
                " must be an enum!");
  static const std::pair<PigeonIMU::CalibrationMode, BasicJsonType> m[] = {
    { PigeonIMU::CalibrationMode::BootTareGyroAccel, "BootTareGyroAccel" },
    { PigeonIMU::CalibrationMode::Temperature, "Temperature" },
    { PigeonIMU::CalibrationMode::Magnetometer12Pt, "Magnetometer12Pt" },
    { PigeonIMU::CalibrationMode::Magnetometer360, "Magnetometer360" },
    { PigeonIMU::CalibrationMode::Accelerometer, "Accelerometer" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<PigeonIMU::CalibrationMode, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.first == e; });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, PigeonIMU::CalibrationMode& e)
{
  static_assert(std::is_enum<PigeonIMU::CalibrationMode>::value,
                "PigeonIMU::CalibrationMode"
                " must be an enum!");
  static const std::pair<PigeonIMU::CalibrationMode, BasicJsonType> m[] = {
    { PigeonIMU::CalibrationMode::BootTareGyroAccel, "BootTareGyroAccel" },
    { PigeonIMU::CalibrationMode::Temperature, "Temperature" },
    { PigeonIMU::CalibrationMode::Magnetometer12Pt, "Magnetometer12Pt" },
    { PigeonIMU::CalibrationMode::Magnetometer360, "Magnetometer360" },
    { PigeonIMU::CalibrationMode::Accelerometer, "Accelerometer" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<PigeonIMU::CalibrationMode, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.second == j; });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

template<typename BasicJsonType>
inline void
to_json(BasicJsonType& j, const PigeonIMU::PigeonState& e)
{
  static_assert(std::is_enum<PigeonIMU::PigeonState>::value,
                "PigeonIMU::PigeonState"
                " must be an enum!");
  static const std::pair<PigeonIMU::PigeonState, BasicJsonType> m[] = {
    { PigeonIMU::PigeonState::NoComm, "NoComm" },
    { PigeonIMU::PigeonState::Initializing, "Initializing" },
    { PigeonIMU::PigeonState::Ready, "Ready" },
    { PigeonIMU::PigeonState::UserCalibration, "UserCalibration" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [e](const std::pair<PigeonIMU::PigeonState, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.first == e; });
  j = ((it != std::end(m)) ? it : std::begin(m))->second;
}
template<typename BasicJsonType>
inline void
from_json(const BasicJsonType& j, PigeonIMU::PigeonState& e)
{
  static_assert(std::is_enum<PigeonIMU::PigeonState>::value,
                "PigeonIMU::PigeonState"
                " must be an enum!");
  static const std::pair<PigeonIMU::PigeonState, BasicJsonType> m[] = {
    { PigeonIMU::PigeonState::NoComm, "NoComm" },
    { PigeonIMU::PigeonState::Initializing, "Initializing" },
    { PigeonIMU::PigeonState::Ready, "Ready" },
    { PigeonIMU::PigeonState::UserCalibration, "UserCalibration" }
  };
  auto it = std::find_if(
    std::begin(m),
    std::end(m),
    [&j](const std::pair<PigeonIMU::PigeonState, BasicJsonType>& ej_pair)
      -> bool { return ej_pair.second == j; });
  e = ((it != std::end(m)) ? it : std::begin(m))->first;
}

}
}
}

namespace ctre {
namespace phoenix {
inline void
to_json(wpi::json& nlohmann_json_j,
        const CustomParamConfiguration& nlohmann_json_t)
{
  nlohmann_json_j["customParam0"] = nlohmann_json_t.customParam0;
  nlohmann_json_j["customParam1"] = nlohmann_json_t.customParam1;
  nlohmann_json_j["enableOptimizations"] = nlohmann_json_t.enableOptimizations;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          CustomParamConfiguration& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.customParam0), void>::from_json(
    nlohmann_json_j.at("customParam0"), nlohmann_json_t.customParam0);
  wpi::adl_serializer<decltype(nlohmann_json_t.customParam1), void>::from_json(
    nlohmann_json_j.at("customParam1"), nlohmann_json_t.customParam1);
  wpi::adl_serializer<decltype(nlohmann_json_t.enableOptimizations),
                      void>::from_json(nlohmann_json_j
                                         .at("enableOptimizations"),
                                       nlohmann_json_t.enableOptimizations);
}

inline void
to_json(wpi::json& nlohmann_json_j,
        const CANifierConfiguration& nlohmann_json_t)
{
  nlohmann_json_j["customParam0"] = nlohmann_json_t.customParam0;
  nlohmann_json_j["customParam1"] = nlohmann_json_t.customParam1;
  nlohmann_json_j["enableOptimizations"] = nlohmann_json_t.enableOptimizations;
  nlohmann_json_j["velocityMeasurementPeriod"] =
    nlohmann_json_t.velocityMeasurementPeriod;
  nlohmann_json_j["velocityMeasurementWindow"] =
    nlohmann_json_t.velocityMeasurementWindow;
  nlohmann_json_j["clearPositionOnLimitF"] =
    nlohmann_json_t.clearPositionOnLimitF;
  nlohmann_json_j["clearPositionOnLimitR"] =
    nlohmann_json_t.clearPositionOnLimitR;
  nlohmann_json_j["clearPositionOnQuadIdx"] =
    nlohmann_json_t.clearPositionOnQuadIdx;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          CANifierConfiguration& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.customParam0), void>::from_json(
    nlohmann_json_j.at("customParam0"), nlohmann_json_t.customParam0);
  wpi::adl_serializer<decltype(nlohmann_json_t.customParam1), void>::from_json(
    nlohmann_json_j.at("customParam1"), nlohmann_json_t.customParam1);
  wpi::adl_serializer<decltype(nlohmann_json_t.enableOptimizations),
                      void>::from_json(nlohmann_json_j
                                         .at("enableOptimizations"),
                                       nlohmann_json_t.enableOptimizations);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.velocityMeasurementPeriod),
    void>::from_json(nlohmann_json_j.at("velocityMeasurementPeriod"),
                     nlohmann_json_t.velocityMeasurementPeriod);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.velocityMeasurementWindow),
    void>::from_json(nlohmann_json_j.at("velocityMeasurementWindow"),
                     nlohmann_json_t.velocityMeasurementWindow);
  wpi::adl_serializer<decltype(nlohmann_json_t.clearPositionOnLimitF),
                      void>::from_json(nlohmann_json_j
                                         .at("clearPositionOnLimitF"),
                                       nlohmann_json_t.clearPositionOnLimitF);
  wpi::adl_serializer<decltype(nlohmann_json_t.clearPositionOnLimitR),
                      void>::from_json(nlohmann_json_j
                                         .at("clearPositionOnLimitR"),
                                       nlohmann_json_t.clearPositionOnLimitR);
  wpi::adl_serializer<decltype(nlohmann_json_t.clearPositionOnQuadIdx),
                      void>::from_json(nlohmann_json_j
                                         .at("clearPositionOnQuadIdx"),
                                       nlohmann_json_t.clearPositionOnQuadIdx);
}

inline void
to_json(wpi::json& nlohmann_json_j, const CANifier::PinValues& nlohmann_json_t)
{
  nlohmann_json_j["QUAD_IDX"] = nlohmann_json_t.QUAD_IDX;
  nlohmann_json_j["QUAD_B"] = nlohmann_json_t.QUAD_B;
  nlohmann_json_j["QUAD_A"] = nlohmann_json_t.QUAD_A;
  nlohmann_json_j["LIMR"] = nlohmann_json_t.LIMR;
  nlohmann_json_j["LIMF"] = nlohmann_json_t.LIMF;
  nlohmann_json_j["SDA"] = nlohmann_json_t.SDA;
  nlohmann_json_j["SCL"] = nlohmann_json_t.SCL;
  nlohmann_json_j["SPI_CS_PWM3"] = nlohmann_json_t.SPI_CS_PWM3;
  nlohmann_json_j["SPI_MISO_PWM2"] = nlohmann_json_t.SPI_MISO_PWM2;
  nlohmann_json_j["SPI_MOSI_PWM1"] = nlohmann_json_t.SPI_MOSI_PWM1;
  nlohmann_json_j["SPI_CLK_PWM0"] = nlohmann_json_t.SPI_CLK_PWM0;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          CANifier::PinValues& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.QUAD_IDX), void>::from_json(
    nlohmann_json_j.at("QUAD_IDX"), nlohmann_json_t.QUAD_IDX);
  wpi::adl_serializer<decltype(nlohmann_json_t.QUAD_B), void>::from_json(
    nlohmann_json_j.at("QUAD_B"), nlohmann_json_t.QUAD_B);
  wpi::adl_serializer<decltype(nlohmann_json_t.QUAD_A), void>::from_json(
    nlohmann_json_j.at("QUAD_A"), nlohmann_json_t.QUAD_A);
  wpi::adl_serializer<decltype(nlohmann_json_t.LIMR), void>::from_json(
    nlohmann_json_j.at("LIMR"), nlohmann_json_t.LIMR);
  wpi::adl_serializer<decltype(nlohmann_json_t.LIMF), void>::from_json(
    nlohmann_json_j.at("LIMF"), nlohmann_json_t.LIMF);
  wpi::adl_serializer<decltype(nlohmann_json_t.SDA), void>::from_json(
    nlohmann_json_j.at("SDA"), nlohmann_json_t.SDA);
  wpi::adl_serializer<decltype(nlohmann_json_t.SCL), void>::from_json(
    nlohmann_json_j.at("SCL"), nlohmann_json_t.SCL);
  wpi::adl_serializer<decltype(nlohmann_json_t.SPI_CS_PWM3), void>::from_json(
    nlohmann_json_j.at("SPI_CS_PWM3"), nlohmann_json_t.SPI_CS_PWM3);
  wpi::adl_serializer<decltype(nlohmann_json_t.SPI_MISO_PWM2), void>::from_json(
    nlohmann_json_j.at("SPI_MISO_PWM2"), nlohmann_json_t.SPI_MISO_PWM2);
  wpi::adl_serializer<decltype(nlohmann_json_t.SPI_MOSI_PWM1), void>::from_json(
    nlohmann_json_j.at("SPI_MOSI_PWM1"), nlohmann_json_t.SPI_MOSI_PWM1);
  wpi::adl_serializer<decltype(nlohmann_json_t.SPI_CLK_PWM0), void>::from_json(
    nlohmann_json_j.at("SPI_CLK_PWM0"), nlohmann_json_t.SPI_CLK_PWM0);
}

namespace motion {
inline void
to_json(wpi::json& nlohmann_json_j, const TrajectoryPoint& nlohmann_json_t)
{
  nlohmann_json_j["position"] = nlohmann_json_t.position;
  nlohmann_json_j["velocity"] = nlohmann_json_t.velocity;
  nlohmann_json_j["arbFeedFwd"] = nlohmann_json_t.arbFeedFwd;
  nlohmann_json_j["auxiliaryPos"] = nlohmann_json_t.auxiliaryPos;
  nlohmann_json_j["auxiliaryVel"] = nlohmann_json_t.auxiliaryVel;
  nlohmann_json_j["auxiliaryArbFeedFwd"] = nlohmann_json_t.auxiliaryArbFeedFwd;
  nlohmann_json_j["profileSlotSelect0"] = nlohmann_json_t.profileSlotSelect0;
  nlohmann_json_j["profileSlotSelect1"] = nlohmann_json_t.profileSlotSelect1;
  nlohmann_json_j["isLastPoint"] = nlohmann_json_t.isLastPoint;
  nlohmann_json_j["zeroPos"] = nlohmann_json_t.zeroPos;
  nlohmann_json_j["timeDur"] = nlohmann_json_t.timeDur;
  nlohmann_json_j["useAuxPID"] = nlohmann_json_t.useAuxPID;
}
inline void
from_json(const wpi::json& nlohmann_json_j, TrajectoryPoint& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.position), void>::from_json(
    nlohmann_json_j.at("position"), nlohmann_json_t.position);
  wpi::adl_serializer<decltype(nlohmann_json_t.velocity), void>::from_json(
    nlohmann_json_j.at("velocity"), nlohmann_json_t.velocity);
  wpi::adl_serializer<decltype(nlohmann_json_t.arbFeedFwd), void>::from_json(
    nlohmann_json_j.at("arbFeedFwd"), nlohmann_json_t.arbFeedFwd);
  wpi::adl_serializer<decltype(nlohmann_json_t.auxiliaryPos), void>::from_json(
    nlohmann_json_j.at("auxiliaryPos"), nlohmann_json_t.auxiliaryPos);
  wpi::adl_serializer<decltype(nlohmann_json_t.auxiliaryVel), void>::from_json(
    nlohmann_json_j.at("auxiliaryVel"), nlohmann_json_t.auxiliaryVel);
  wpi::adl_serializer<decltype(nlohmann_json_t.auxiliaryArbFeedFwd),
                      void>::from_json(nlohmann_json_j
                                         .at("auxiliaryArbFeedFwd"),
                                       nlohmann_json_t.auxiliaryArbFeedFwd);
  wpi::adl_serializer<decltype(nlohmann_json_t.profileSlotSelect0),
                      void>::from_json(nlohmann_json_j.at("profileSlotSelect0"),
                                       nlohmann_json_t.profileSlotSelect0);
  wpi::adl_serializer<decltype(nlohmann_json_t.profileSlotSelect1),
                      void>::from_json(nlohmann_json_j.at("profileSlotSelect1"),
                                       nlohmann_json_t.profileSlotSelect1);
  wpi::adl_serializer<decltype(nlohmann_json_t.isLastPoint), void>::from_json(
    nlohmann_json_j.at("isLastPoint"), nlohmann_json_t.isLastPoint);
  wpi::adl_serializer<decltype(nlohmann_json_t.zeroPos), void>::from_json(
    nlohmann_json_j.at("zeroPos"), nlohmann_json_t.zeroPos);
  wpi::adl_serializer<decltype(nlohmann_json_t.timeDur), void>::from_json(
    nlohmann_json_j.at("timeDur"), nlohmann_json_t.timeDur);
  wpi::adl_serializer<decltype(nlohmann_json_t.useAuxPID), void>::from_json(
    nlohmann_json_j.at("useAuxPID"), nlohmann_json_t.useAuxPID);
}

inline void
to_json(wpi::json& nlohmann_json_j, const MotionProfileStatus& nlohmann_json_t)
{
  nlohmann_json_j["topBufferRem"] = nlohmann_json_t.topBufferRem;
  nlohmann_json_j["topBufferCnt"] = nlohmann_json_t.topBufferCnt;
  nlohmann_json_j["btmBufferCnt"] = nlohmann_json_t.btmBufferCnt;
  nlohmann_json_j["hasUnderrun"] = nlohmann_json_t.hasUnderrun;
  nlohmann_json_j["isUnderrun"] = nlohmann_json_t.isUnderrun;
  nlohmann_json_j["activePointValid"] = nlohmann_json_t.activePointValid;
  nlohmann_json_j["isLast"] = nlohmann_json_t.isLast;
  nlohmann_json_j["profileSlotSelect0"] = nlohmann_json_t.profileSlotSelect0;
  nlohmann_json_j["profileSlotSelect1"] = nlohmann_json_t.profileSlotSelect1;
  nlohmann_json_j["outputEnable"] = nlohmann_json_t.outputEnable;
  nlohmann_json_j["timeDurMs"] = nlohmann_json_t.timeDurMs;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          MotionProfileStatus& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.topBufferRem), void>::from_json(
    nlohmann_json_j.at("topBufferRem"), nlohmann_json_t.topBufferRem);
  wpi::adl_serializer<decltype(nlohmann_json_t.topBufferCnt), void>::from_json(
    nlohmann_json_j.at("topBufferCnt"), nlohmann_json_t.topBufferCnt);
  wpi::adl_serializer<decltype(nlohmann_json_t.btmBufferCnt), void>::from_json(
    nlohmann_json_j.at("btmBufferCnt"), nlohmann_json_t.btmBufferCnt);
  wpi::adl_serializer<decltype(nlohmann_json_t.hasUnderrun), void>::from_json(
    nlohmann_json_j.at("hasUnderrun"), nlohmann_json_t.hasUnderrun);
  wpi::adl_serializer<decltype(nlohmann_json_t.isUnderrun), void>::from_json(
    nlohmann_json_j.at("isUnderrun"), nlohmann_json_t.isUnderrun);
  wpi::adl_serializer<decltype(nlohmann_json_t.activePointValid),
                      void>::from_json(nlohmann_json_j.at("activePointValid"),
                                       nlohmann_json_t.activePointValid);
  wpi::adl_serializer<decltype(nlohmann_json_t.isLast), void>::from_json(
    nlohmann_json_j.at("isLast"), nlohmann_json_t.isLast);
  wpi::adl_serializer<decltype(nlohmann_json_t.profileSlotSelect0),
                      void>::from_json(nlohmann_json_j.at("profileSlotSelect0"),
                                       nlohmann_json_t.profileSlotSelect0);
  wpi::adl_serializer<decltype(nlohmann_json_t.profileSlotSelect1),
                      void>::from_json(nlohmann_json_j.at("profileSlotSelect1"),
                                       nlohmann_json_t.profileSlotSelect1);
  wpi::adl_serializer<decltype(nlohmann_json_t.outputEnable), void>::from_json(
    nlohmann_json_j.at("outputEnable"), nlohmann_json_t.outputEnable);
  wpi::adl_serializer<decltype(nlohmann_json_t.timeDurMs), void>::from_json(
    nlohmann_json_j.at("timeDurMs"), nlohmann_json_t.timeDurMs);
}

}
namespace motorcontrol {

inline void
to_json(wpi::json& nlohmann_json_j, const Faults& nlohmann_json_t)
{
  nlohmann_json_j["UnderVoltage"] = nlohmann_json_t.UnderVoltage;
  nlohmann_json_j["ForwardLimitSwitch"] = nlohmann_json_t.ForwardLimitSwitch;
  nlohmann_json_j["ReverseLimitSwitch"] = nlohmann_json_t.ReverseLimitSwitch;
  nlohmann_json_j["ForwardSoftLimit"] = nlohmann_json_t.ForwardSoftLimit;
  nlohmann_json_j["ReverseSoftLimit"] = nlohmann_json_t.ReverseSoftLimit;
  nlohmann_json_j["HardwareFailure"] = nlohmann_json_t.HardwareFailure;
  nlohmann_json_j["ResetDuringEn"] = nlohmann_json_t.ResetDuringEn;
  nlohmann_json_j["SensorOverflow"] = nlohmann_json_t.SensorOverflow;
  nlohmann_json_j["SensorOutOfPhase"] = nlohmann_json_t.SensorOutOfPhase;
  nlohmann_json_j["HardwareESDReset"] = nlohmann_json_t.HardwareESDReset;
  nlohmann_json_j["RemoteLossOfSignal"] = nlohmann_json_t.RemoteLossOfSignal;
  nlohmann_json_j["APIError"] = nlohmann_json_t.APIError;
  nlohmann_json_j["SupplyOverV"] = nlohmann_json_t.SupplyOverV;
  nlohmann_json_j["SupplyUnstable"] = nlohmann_json_t.SupplyUnstable;
}
inline void
from_json(const wpi::json& nlohmann_json_j, Faults& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.UnderVoltage), void>::from_json(
    nlohmann_json_j.at("UnderVoltage"), nlohmann_json_t.UnderVoltage);
  wpi::adl_serializer<decltype(nlohmann_json_t.ForwardLimitSwitch),
                      void>::from_json(nlohmann_json_j.at("ForwardLimitSwitch"),
                                       nlohmann_json_t.ForwardLimitSwitch);
  wpi::adl_serializer<decltype(nlohmann_json_t.ReverseLimitSwitch),
                      void>::from_json(nlohmann_json_j.at("ReverseLimitSwitch"),
                                       nlohmann_json_t.ReverseLimitSwitch);
  wpi::adl_serializer<decltype(nlohmann_json_t.ForwardSoftLimit),
                      void>::from_json(nlohmann_json_j.at("ForwardSoftLimit"),
                                       nlohmann_json_t.ForwardSoftLimit);
  wpi::adl_serializer<decltype(nlohmann_json_t.ReverseSoftLimit),
                      void>::from_json(nlohmann_json_j.at("ReverseSoftLimit"),
                                       nlohmann_json_t.ReverseSoftLimit);
  wpi::adl_serializer<decltype(nlohmann_json_t.HardwareFailure),
                      void>::from_json(nlohmann_json_j.at("HardwareFailure"),
                                       nlohmann_json_t.HardwareFailure);
  wpi::adl_serializer<decltype(nlohmann_json_t.ResetDuringEn), void>::from_json(
    nlohmann_json_j.at("ResetDuringEn"), nlohmann_json_t.ResetDuringEn);
  wpi::adl_serializer<decltype(nlohmann_json_t.SensorOverflow),
                      void>::from_json(nlohmann_json_j.at("SensorOverflow"),
                                       nlohmann_json_t.SensorOverflow);
  wpi::adl_serializer<decltype(nlohmann_json_t.SensorOutOfPhase),
                      void>::from_json(nlohmann_json_j.at("SensorOutOfPhase"),
                                       nlohmann_json_t.SensorOutOfPhase);
  wpi::adl_serializer<decltype(nlohmann_json_t.HardwareESDReset),
                      void>::from_json(nlohmann_json_j.at("HardwareESDReset"),
                                       nlohmann_json_t.HardwareESDReset);
  wpi::adl_serializer<decltype(nlohmann_json_t.RemoteLossOfSignal),
                      void>::from_json(nlohmann_json_j.at("RemoteLossOfSignal"),
                                       nlohmann_json_t.RemoteLossOfSignal);
  wpi::adl_serializer<decltype(nlohmann_json_t.APIError), void>::from_json(
    nlohmann_json_j.at("APIError"), nlohmann_json_t.APIError);
  wpi::adl_serializer<decltype(nlohmann_json_t.SupplyOverV), void>::from_json(
    nlohmann_json_j.at("SupplyOverV"), nlohmann_json_t.SupplyOverV);
  wpi::adl_serializer<decltype(nlohmann_json_t.SupplyUnstable),
                      void>::from_json(nlohmann_json_j.at("SupplyUnstable"),
                                       nlohmann_json_t.SupplyUnstable);
}

inline void
to_json(wpi::json& nlohmann_json_j, const StickyFaults& nlohmann_json_t)
{
  nlohmann_json_j["UnderVoltage"] = nlohmann_json_t.UnderVoltage;
  nlohmann_json_j["ForwardLimitSwitch"] = nlohmann_json_t.ForwardLimitSwitch;
  nlohmann_json_j["ReverseLimitSwitch"] = nlohmann_json_t.ReverseLimitSwitch;
  nlohmann_json_j["ForwardSoftLimit"] = nlohmann_json_t.ForwardSoftLimit;
  nlohmann_json_j["ReverseSoftLimit"] = nlohmann_json_t.ReverseSoftLimit;
  nlohmann_json_j["ResetDuringEn"] = nlohmann_json_t.ResetDuringEn;
  nlohmann_json_j["SensorOverflow"] = nlohmann_json_t.SensorOverflow;
  nlohmann_json_j["SensorOutOfPhase"] = nlohmann_json_t.SensorOutOfPhase;
  nlohmann_json_j["HardwareESDReset"] = nlohmann_json_t.HardwareESDReset;
  nlohmann_json_j["RemoteLossOfSignal"] = nlohmann_json_t.RemoteLossOfSignal;
  nlohmann_json_j["APIError"] = nlohmann_json_t.APIError;
  nlohmann_json_j["SupplyOverV"] = nlohmann_json_t.SupplyOverV;
  nlohmann_json_j["SupplyUnstable"] = nlohmann_json_t.SupplyUnstable;
}
inline void
from_json(const wpi::json& nlohmann_json_j, StickyFaults& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.UnderVoltage), void>::from_json(
    nlohmann_json_j.at("UnderVoltage"), nlohmann_json_t.UnderVoltage);
  wpi::adl_serializer<decltype(nlohmann_json_t.ForwardLimitSwitch),
                      void>::from_json(nlohmann_json_j.at("ForwardLimitSwitch"),
                                       nlohmann_json_t.ForwardLimitSwitch);
  wpi::adl_serializer<decltype(nlohmann_json_t.ReverseLimitSwitch),
                      void>::from_json(nlohmann_json_j.at("ReverseLimitSwitch"),
                                       nlohmann_json_t.ReverseLimitSwitch);
  wpi::adl_serializer<decltype(nlohmann_json_t.ForwardSoftLimit),
                      void>::from_json(nlohmann_json_j.at("ForwardSoftLimit"),
                                       nlohmann_json_t.ForwardSoftLimit);
  wpi::adl_serializer<decltype(nlohmann_json_t.ReverseSoftLimit),
                      void>::from_json(nlohmann_json_j.at("ReverseSoftLimit"),
                                       nlohmann_json_t.ReverseSoftLimit);
  wpi::adl_serializer<decltype(nlohmann_json_t.ResetDuringEn), void>::from_json(
    nlohmann_json_j.at("ResetDuringEn"), nlohmann_json_t.ResetDuringEn);
  wpi::adl_serializer<decltype(nlohmann_json_t.SensorOverflow),
                      void>::from_json(nlohmann_json_j.at("SensorOverflow"),
                                       nlohmann_json_t.SensorOverflow);
  wpi::adl_serializer<decltype(nlohmann_json_t.SensorOutOfPhase),
                      void>::from_json(nlohmann_json_j.at("SensorOutOfPhase"),
                                       nlohmann_json_t.SensorOutOfPhase);
  wpi::adl_serializer<decltype(nlohmann_json_t.HardwareESDReset),
                      void>::from_json(nlohmann_json_j.at("HardwareESDReset"),
                                       nlohmann_json_t.HardwareESDReset);
  wpi::adl_serializer<decltype(nlohmann_json_t.RemoteLossOfSignal),
                      void>::from_json(nlohmann_json_j.at("RemoteLossOfSignal"),
                                       nlohmann_json_t.RemoteLossOfSignal);
  wpi::adl_serializer<decltype(nlohmann_json_t.APIError), void>::from_json(
    nlohmann_json_j.at("APIError"), nlohmann_json_t.APIError);
  wpi::adl_serializer<decltype(nlohmann_json_t.SupplyOverV), void>::from_json(
    nlohmann_json_j.at("SupplyOverV"), nlohmann_json_t.SupplyOverV);
  wpi::adl_serializer<decltype(nlohmann_json_t.SupplyUnstable),
                      void>::from_json(nlohmann_json_j.at("SupplyUnstable"),
                                       nlohmann_json_t.SupplyUnstable);
}

inline void
to_json(wpi::json& nlohmann_json_j,
        const StatorCurrentLimitConfiguration& nlohmann_json_t)
{
  nlohmann_json_j["enable"] = nlohmann_json_t.enable;
  nlohmann_json_j["currentLimit"] = nlohmann_json_t.currentLimit;
  nlohmann_json_j["triggerThresholdCurrent"] =
    nlohmann_json_t.triggerThresholdCurrent;
  nlohmann_json_j["triggerThresholdTime"] =
    nlohmann_json_t.triggerThresholdTime;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          StatorCurrentLimitConfiguration& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.enable), void>::from_json(
    nlohmann_json_j.at("enable"), nlohmann_json_t.enable);
  wpi::adl_serializer<decltype(nlohmann_json_t.currentLimit), void>::from_json(
    nlohmann_json_j.at("currentLimit"), nlohmann_json_t.currentLimit);
  wpi::adl_serializer<decltype(nlohmann_json_t.triggerThresholdCurrent),
                      void>::from_json(nlohmann_json_j
                                         .at("triggerThresholdCurrent"),
                                       nlohmann_json_t.triggerThresholdCurrent);
  wpi::adl_serializer<decltype(nlohmann_json_t.triggerThresholdTime),
                      void>::from_json(nlohmann_json_j
                                         .at("triggerThresholdTime"),
                                       nlohmann_json_t.triggerThresholdTime);
}

inline void
to_json(wpi::json& nlohmann_json_j,
        const SupplyCurrentLimitConfiguration& nlohmann_json_t)
{
  nlohmann_json_j["enable"] = nlohmann_json_t.enable;
  nlohmann_json_j["currentLimit"] = nlohmann_json_t.currentLimit;
  nlohmann_json_j["triggerThresholdCurrent"] =
    nlohmann_json_t.triggerThresholdCurrent;
  nlohmann_json_j["triggerThresholdTime"] =
    nlohmann_json_t.triggerThresholdTime;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          SupplyCurrentLimitConfiguration& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.enable), void>::from_json(
    nlohmann_json_j.at("enable"), nlohmann_json_t.enable);
  wpi::adl_serializer<decltype(nlohmann_json_t.currentLimit), void>::from_json(
    nlohmann_json_j.at("currentLimit"), nlohmann_json_t.currentLimit);
  wpi::adl_serializer<decltype(nlohmann_json_t.triggerThresholdCurrent),
                      void>::from_json(nlohmann_json_j
                                         .at("triggerThresholdCurrent"),
                                       nlohmann_json_t.triggerThresholdCurrent);
  wpi::adl_serializer<decltype(nlohmann_json_t.triggerThresholdTime),
                      void>::from_json(nlohmann_json_j
                                         .at("triggerThresholdTime"),
                                       nlohmann_json_t.triggerThresholdTime);
}

namespace can {
inline void
to_json(wpi::json& nlohmann_json_j,
        const BasePIDSetConfiguration& nlohmann_json_t)
{
  nlohmann_json_j["selectedFeedbackCoefficient"] =
    nlohmann_json_t.selectedFeedbackCoefficient;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          BasePIDSetConfiguration& nlohmann_json_t)
{
  wpi::adl_serializer<
    decltype(nlohmann_json_t.selectedFeedbackCoefficient),
    void>::from_json(nlohmann_json_j.at("selectedFeedbackCoefficient"),
                     nlohmann_json_t.selectedFeedbackCoefficient);
}

inline void
to_json(wpi::json& nlohmann_json_j, const FilterConfiguration& nlohmann_json_t)
{
  nlohmann_json_j["remoteSensorDeviceID"] =
    nlohmann_json_t.remoteSensorDeviceID;
  nlohmann_json_j["remoteSensorSource"] = nlohmann_json_t.remoteSensorSource;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          FilterConfiguration& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.remoteSensorDeviceID),
                      void>::from_json(nlohmann_json_j
                                         .at("remoteSensorDeviceID"),
                                       nlohmann_json_t.remoteSensorDeviceID);
  wpi::adl_serializer<decltype(nlohmann_json_t.remoteSensorSource),
                      void>::from_json(nlohmann_json_j.at("remoteSensorSource"),
                                       nlohmann_json_t.remoteSensorSource);
}

inline void
to_json(wpi::json& nlohmann_json_j, const SlotConfiguration& nlohmann_json_t)
{
  nlohmann_json_j["kP"] = nlohmann_json_t.kP;
  nlohmann_json_j["kI"] = nlohmann_json_t.kI;
  nlohmann_json_j["kD"] = nlohmann_json_t.kD;
  nlohmann_json_j["kF"] = nlohmann_json_t.kF;
  nlohmann_json_j["integralZone"] = nlohmann_json_t.integralZone;
  nlohmann_json_j["allowableClosedloopError"] =
    nlohmann_json_t.allowableClosedloopError;
  nlohmann_json_j["maxIntegralAccumulator"] =
    nlohmann_json_t.maxIntegralAccumulator;
  nlohmann_json_j["closedLoopPeakOutput"] =
    nlohmann_json_t.closedLoopPeakOutput;
  nlohmann_json_j["closedLoopPeriod"] = nlohmann_json_t.closedLoopPeriod;
}
inline void
from_json(const wpi::json& nlohmann_json_j, SlotConfiguration& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.kP), void>::from_json(
    nlohmann_json_j.at("kP"), nlohmann_json_t.kP);
  wpi::adl_serializer<decltype(nlohmann_json_t.kI), void>::from_json(
    nlohmann_json_j.at("kI"), nlohmann_json_t.kI);
  wpi::adl_serializer<decltype(nlohmann_json_t.kD), void>::from_json(
    nlohmann_json_j.at("kD"), nlohmann_json_t.kD);
  wpi::adl_serializer<decltype(nlohmann_json_t.kF), void>::from_json(
    nlohmann_json_j.at("kF"), nlohmann_json_t.kF);
  wpi::adl_serializer<decltype(nlohmann_json_t.integralZone), void>::from_json(
    nlohmann_json_j.at("integralZone"), nlohmann_json_t.integralZone);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.allowableClosedloopError),
    void>::from_json(nlohmann_json_j.at("allowableClosedloopError"),
                     nlohmann_json_t.allowableClosedloopError);
  wpi::adl_serializer<decltype(nlohmann_json_t.maxIntegralAccumulator),
                      void>::from_json(nlohmann_json_j
                                         .at("maxIntegralAccumulator"),
                                       nlohmann_json_t.maxIntegralAccumulator);
  wpi::adl_serializer<decltype(nlohmann_json_t.closedLoopPeakOutput),
                      void>::from_json(nlohmann_json_j
                                         .at("closedLoopPeakOutput"),
                                       nlohmann_json_t.closedLoopPeakOutput);
  wpi::adl_serializer<decltype(nlohmann_json_t.closedLoopPeriod),
                      void>::from_json(nlohmann_json_j.at("closedLoopPeriod"),
                                       nlohmann_json_t.closedLoopPeriod);
}

inline void
to_json(wpi::json& nlohmann_json_j,
        const BaseMotorControllerConfiguration& nlohmann_json_t)
{
  nlohmann_json_j["customParam0"] = nlohmann_json_t.customParam0;
  nlohmann_json_j["customParam1"] = nlohmann_json_t.customParam1;
  nlohmann_json_j["enableOptimizations"] = nlohmann_json_t.enableOptimizations;
  nlohmann_json_j["openloopRamp"] = nlohmann_json_t.openloopRamp;
  nlohmann_json_j["closedloopRamp"] = nlohmann_json_t.closedloopRamp;
  nlohmann_json_j["peakOutputForward"] = nlohmann_json_t.peakOutputForward;
  nlohmann_json_j["peakOutputReverse"] = nlohmann_json_t.peakOutputReverse;
  nlohmann_json_j["nominalOutputForward"] =
    nlohmann_json_t.nominalOutputForward;
  nlohmann_json_j["nominalOutputReverse"] =
    nlohmann_json_t.nominalOutputReverse;
  nlohmann_json_j["neutralDeadband"] = nlohmann_json_t.neutralDeadband;
  nlohmann_json_j["voltageCompSaturation"] =
    nlohmann_json_t.voltageCompSaturation;
  nlohmann_json_j["voltageMeasurementFilter"] =
    nlohmann_json_t.voltageMeasurementFilter;
  nlohmann_json_j["velocityMeasurementPeriod"] =
    nlohmann_json_t.velocityMeasurementPeriod;
  nlohmann_json_j["velocityMeasurementWindow"] =
    nlohmann_json_t.velocityMeasurementWindow;
  nlohmann_json_j["forwardSoftLimitThreshold"] =
    nlohmann_json_t.forwardSoftLimitThreshold;
  nlohmann_json_j["reverseSoftLimitThreshold"] =
    nlohmann_json_t.reverseSoftLimitThreshold;
  nlohmann_json_j["forwardSoftLimitEnable"] =
    nlohmann_json_t.forwardSoftLimitEnable;
  nlohmann_json_j["reverseSoftLimitEnable"] =
    nlohmann_json_t.reverseSoftLimitEnable;
  nlohmann_json_j["slot0"] = nlohmann_json_t.slot0;
  nlohmann_json_j["slot1"] = nlohmann_json_t.slot1;
  nlohmann_json_j["slot2"] = nlohmann_json_t.slot2;
  nlohmann_json_j["slot3"] = nlohmann_json_t.slot3;
  nlohmann_json_j["auxPIDPolarity"] = nlohmann_json_t.auxPIDPolarity;
  nlohmann_json_j["remoteFilter0"] = nlohmann_json_t.remoteFilter0;
  nlohmann_json_j["remoteFilter1"] = nlohmann_json_t.remoteFilter1;
  nlohmann_json_j["motionCruiseVelocity"] =
    nlohmann_json_t.motionCruiseVelocity;
  nlohmann_json_j["motionAcceleration"] = nlohmann_json_t.motionAcceleration;
  nlohmann_json_j["motionCurveStrength"] = nlohmann_json_t.motionCurveStrength;
  nlohmann_json_j["motionProfileTrajectoryPeriod"] =
    nlohmann_json_t.motionProfileTrajectoryPeriod;
  nlohmann_json_j["feedbackNotContinuous"] =
    nlohmann_json_t.feedbackNotContinuous;
  nlohmann_json_j["remoteSensorClosedLoopDisableNeutralOnLOS"] =
    nlohmann_json_t.remoteSensorClosedLoopDisableNeutralOnLOS;
  nlohmann_json_j["clearPositionOnLimitF"] =
    nlohmann_json_t.clearPositionOnLimitF;
  nlohmann_json_j["clearPositionOnLimitR"] =
    nlohmann_json_t.clearPositionOnLimitR;
  nlohmann_json_j["clearPositionOnQuadIdx"] =
    nlohmann_json_t.clearPositionOnQuadIdx;
  nlohmann_json_j["limitSwitchDisableNeutralOnLOS"] =
    nlohmann_json_t.limitSwitchDisableNeutralOnLOS;
  nlohmann_json_j["softLimitDisableNeutralOnLOS"] =
    nlohmann_json_t.softLimitDisableNeutralOnLOS;
  nlohmann_json_j["pulseWidthPeriod_EdgesPerRot"] =
    nlohmann_json_t.pulseWidthPeriod_EdgesPerRot;
  nlohmann_json_j["pulseWidthPeriod_FilterWindowSz"] =
    nlohmann_json_t.pulseWidthPeriod_FilterWindowSz;
  nlohmann_json_j["trajectoryInterpolationEnable"] =
    nlohmann_json_t.trajectoryInterpolationEnable;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          BaseMotorControllerConfiguration& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.customParam0), void>::from_json(
    nlohmann_json_j.at("customParam0"), nlohmann_json_t.customParam0);
  wpi::adl_serializer<decltype(nlohmann_json_t.customParam1), void>::from_json(
    nlohmann_json_j.at("customParam1"), nlohmann_json_t.customParam1);
  wpi::adl_serializer<decltype(nlohmann_json_t.enableOptimizations),
                      void>::from_json(nlohmann_json_j
                                         .at("enableOptimizations"),
                                       nlohmann_json_t.enableOptimizations);
  wpi::adl_serializer<decltype(nlohmann_json_t.openloopRamp), void>::from_json(
    nlohmann_json_j.at("openloopRamp"), nlohmann_json_t.openloopRamp);
  wpi::adl_serializer<decltype(nlohmann_json_t.closedloopRamp),
                      void>::from_json(nlohmann_json_j.at("closedloopRamp"),
                                       nlohmann_json_t.closedloopRamp);
  wpi::adl_serializer<decltype(nlohmann_json_t.peakOutputForward),
                      void>::from_json(nlohmann_json_j.at("peakOutputForward"),
                                       nlohmann_json_t.peakOutputForward);
  wpi::adl_serializer<decltype(nlohmann_json_t.peakOutputReverse),
                      void>::from_json(nlohmann_json_j.at("peakOutputReverse"),
                                       nlohmann_json_t.peakOutputReverse);
  wpi::adl_serializer<decltype(nlohmann_json_t.nominalOutputForward),
                      void>::from_json(nlohmann_json_j
                                         .at("nominalOutputForward"),
                                       nlohmann_json_t.nominalOutputForward);
  wpi::adl_serializer<decltype(nlohmann_json_t.nominalOutputReverse),
                      void>::from_json(nlohmann_json_j
                                         .at("nominalOutputReverse"),
                                       nlohmann_json_t.nominalOutputReverse);
  wpi::adl_serializer<decltype(nlohmann_json_t.neutralDeadband),
                      void>::from_json(nlohmann_json_j.at("neutralDeadband"),
                                       nlohmann_json_t.neutralDeadband);
  wpi::adl_serializer<decltype(nlohmann_json_t.voltageCompSaturation),
                      void>::from_json(nlohmann_json_j
                                         .at("voltageCompSaturation"),
                                       nlohmann_json_t.voltageCompSaturation);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.voltageMeasurementFilter),
    void>::from_json(nlohmann_json_j.at("voltageMeasurementFilter"),
                     nlohmann_json_t.voltageMeasurementFilter);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.velocityMeasurementPeriod),
    void>::from_json(nlohmann_json_j.at("velocityMeasurementPeriod"),
                     nlohmann_json_t.velocityMeasurementPeriod);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.velocityMeasurementWindow),
    void>::from_json(nlohmann_json_j.at("velocityMeasurementWindow"),
                     nlohmann_json_t.velocityMeasurementWindow);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.forwardSoftLimitThreshold),
    void>::from_json(nlohmann_json_j.at("forwardSoftLimitThreshold"),
                     nlohmann_json_t.forwardSoftLimitThreshold);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.reverseSoftLimitThreshold),
    void>::from_json(nlohmann_json_j.at("reverseSoftLimitThreshold"),
                     nlohmann_json_t.reverseSoftLimitThreshold);
  wpi::adl_serializer<decltype(nlohmann_json_t.forwardSoftLimitEnable),
                      void>::from_json(nlohmann_json_j
                                         .at("forwardSoftLimitEnable"),
                                       nlohmann_json_t.forwardSoftLimitEnable);
  wpi::adl_serializer<decltype(nlohmann_json_t.reverseSoftLimitEnable),
                      void>::from_json(nlohmann_json_j
                                         .at("reverseSoftLimitEnable"),
                                       nlohmann_json_t.reverseSoftLimitEnable);
  wpi::adl_serializer<decltype(nlohmann_json_t.slot0), void>::from_json(
    nlohmann_json_j.at("slot0"), nlohmann_json_t.slot0);
  wpi::adl_serializer<decltype(nlohmann_json_t.slot1), void>::from_json(
    nlohmann_json_j.at("slot1"), nlohmann_json_t.slot1);
  wpi::adl_serializer<decltype(nlohmann_json_t.slot2), void>::from_json(
    nlohmann_json_j.at("slot2"), nlohmann_json_t.slot2);
  wpi::adl_serializer<decltype(nlohmann_json_t.slot3), void>::from_json(
    nlohmann_json_j.at("slot3"), nlohmann_json_t.slot3);
  wpi::adl_serializer<decltype(nlohmann_json_t.auxPIDPolarity),
                      void>::from_json(nlohmann_json_j.at("auxPIDPolarity"),
                                       nlohmann_json_t.auxPIDPolarity);
  wpi::adl_serializer<decltype(nlohmann_json_t.remoteFilter0), void>::from_json(
    nlohmann_json_j.at("remoteFilter0"), nlohmann_json_t.remoteFilter0);
  wpi::adl_serializer<decltype(nlohmann_json_t.remoteFilter1), void>::from_json(
    nlohmann_json_j.at("remoteFilter1"), nlohmann_json_t.remoteFilter1);
  wpi::adl_serializer<decltype(nlohmann_json_t.motionCruiseVelocity),
                      void>::from_json(nlohmann_json_j
                                         .at("motionCruiseVelocity"),
                                       nlohmann_json_t.motionCruiseVelocity);
  wpi::adl_serializer<decltype(nlohmann_json_t.motionAcceleration),
                      void>::from_json(nlohmann_json_j.at("motionAcceleration"),
                                       nlohmann_json_t.motionAcceleration);
  wpi::adl_serializer<decltype(nlohmann_json_t.motionCurveStrength),
                      void>::from_json(nlohmann_json_j
                                         .at("motionCurveStrength"),
                                       nlohmann_json_t.motionCurveStrength);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.motionProfileTrajectoryPeriod),
    void>::from_json(nlohmann_json_j.at("motionProfileTrajectoryPeriod"),
                     nlohmann_json_t.motionProfileTrajectoryPeriod);
  wpi::adl_serializer<decltype(nlohmann_json_t.feedbackNotContinuous),
                      void>::from_json(nlohmann_json_j
                                         .at("feedbackNotContinuous"),
                                       nlohmann_json_t.feedbackNotContinuous);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.remoteSensorClosedLoopDisableNeutralOnLOS),
    void>::from_json(nlohmann_json_j
                       .at("remoteSensorClosedLoopDisableNeutralOnLOS"),
                     nlohmann_json_t.remoteSensorClosedLoopDisableNeutralOnLOS);
  wpi::adl_serializer<decltype(nlohmann_json_t.clearPositionOnLimitF),
                      void>::from_json(nlohmann_json_j
                                         .at("clearPositionOnLimitF"),
                                       nlohmann_json_t.clearPositionOnLimitF);
  wpi::adl_serializer<decltype(nlohmann_json_t.clearPositionOnLimitR),
                      void>::from_json(nlohmann_json_j
                                         .at("clearPositionOnLimitR"),
                                       nlohmann_json_t.clearPositionOnLimitR);
  wpi::adl_serializer<decltype(nlohmann_json_t.clearPositionOnQuadIdx),
                      void>::from_json(nlohmann_json_j
                                         .at("clearPositionOnQuadIdx"),
                                       nlohmann_json_t.clearPositionOnQuadIdx);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.limitSwitchDisableNeutralOnLOS),
    void>::from_json(nlohmann_json_j.at("limitSwitchDisableNeutralOnLOS"),
                     nlohmann_json_t.limitSwitchDisableNeutralOnLOS);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.softLimitDisableNeutralOnLOS),
    void>::from_json(nlohmann_json_j.at("softLimitDisableNeutralOnLOS"),
                     nlohmann_json_t.softLimitDisableNeutralOnLOS);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.pulseWidthPeriod_EdgesPerRot),
    void>::from_json(nlohmann_json_j.at("pulseWidthPeriod_EdgesPerRot"),
                     nlohmann_json_t.pulseWidthPeriod_EdgesPerRot);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.pulseWidthPeriod_FilterWindowSz),
    void>::from_json(nlohmann_json_j.at("pulseWidthPeriod_FilterWindowSz"),
                     nlohmann_json_t.pulseWidthPeriod_FilterWindowSz);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.trajectoryInterpolationEnable),
    void>::from_json(nlohmann_json_j.at("trajectoryInterpolationEnable"),
                     nlohmann_json_t.trajectoryInterpolationEnable);
}

inline void
to_json(wpi::json& nlohmann_json_j,
        const BaseTalonPIDSetConfiguration& nlohmann_json_t)
{
  nlohmann_json_j["selectedFeedbackCoefficient"] =
    nlohmann_json_t.selectedFeedbackCoefficient;
  nlohmann_json_j["selectedFeedbackSensor"] =
    nlohmann_json_t.selectedFeedbackSensor;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          BaseTalonPIDSetConfiguration& nlohmann_json_t)
{
  wpi::adl_serializer<
    decltype(nlohmann_json_t.selectedFeedbackCoefficient),
    void>::from_json(nlohmann_json_j.at("selectedFeedbackCoefficient"),
                     nlohmann_json_t.selectedFeedbackCoefficient);
  wpi::adl_serializer<decltype(nlohmann_json_t.selectedFeedbackSensor),
                      void>::from_json(nlohmann_json_j
                                         .at("selectedFeedbackSensor"),
                                       nlohmann_json_t.selectedFeedbackSensor);
}

inline void
to_json(wpi::json& nlohmann_json_j,
        const BaseTalonConfiguration& nlohmann_json_t)
{
  nlohmann_json_j["customParam0"] = nlohmann_json_t.customParam0;
  nlohmann_json_j["customParam1"] = nlohmann_json_t.customParam1;
  nlohmann_json_j["enableOptimizations"] = nlohmann_json_t.enableOptimizations;
  nlohmann_json_j["openloopRamp"] = nlohmann_json_t.openloopRamp;
  nlohmann_json_j["closedloopRamp"] = nlohmann_json_t.closedloopRamp;
  nlohmann_json_j["peakOutputForward"] = nlohmann_json_t.peakOutputForward;
  nlohmann_json_j["peakOutputReverse"] = nlohmann_json_t.peakOutputReverse;
  nlohmann_json_j["nominalOutputForward"] =
    nlohmann_json_t.nominalOutputForward;
  nlohmann_json_j["nominalOutputReverse"] =
    nlohmann_json_t.nominalOutputReverse;
  nlohmann_json_j["neutralDeadband"] = nlohmann_json_t.neutralDeadband;
  nlohmann_json_j["voltageCompSaturation"] =
    nlohmann_json_t.voltageCompSaturation;
  nlohmann_json_j["voltageMeasurementFilter"] =
    nlohmann_json_t.voltageMeasurementFilter;
  nlohmann_json_j["velocityMeasurementPeriod"] =
    nlohmann_json_t.velocityMeasurementPeriod;
  nlohmann_json_j["velocityMeasurementWindow"] =
    nlohmann_json_t.velocityMeasurementWindow;
  nlohmann_json_j["forwardSoftLimitThreshold"] =
    nlohmann_json_t.forwardSoftLimitThreshold;
  nlohmann_json_j["reverseSoftLimitThreshold"] =
    nlohmann_json_t.reverseSoftLimitThreshold;
  nlohmann_json_j["forwardSoftLimitEnable"] =
    nlohmann_json_t.forwardSoftLimitEnable;
  nlohmann_json_j["reverseSoftLimitEnable"] =
    nlohmann_json_t.reverseSoftLimitEnable;
  nlohmann_json_j["slot0"] = nlohmann_json_t.slot0;
  nlohmann_json_j["slot1"] = nlohmann_json_t.slot1;
  nlohmann_json_j["slot2"] = nlohmann_json_t.slot2;
  nlohmann_json_j["slot3"] = nlohmann_json_t.slot3;
  nlohmann_json_j["auxPIDPolarity"] = nlohmann_json_t.auxPIDPolarity;
  nlohmann_json_j["remoteFilter0"] = nlohmann_json_t.remoteFilter0;
  nlohmann_json_j["remoteFilter1"] = nlohmann_json_t.remoteFilter1;
  nlohmann_json_j["motionCruiseVelocity"] =
    nlohmann_json_t.motionCruiseVelocity;
  nlohmann_json_j["motionAcceleration"] = nlohmann_json_t.motionAcceleration;
  nlohmann_json_j["motionCurveStrength"] = nlohmann_json_t.motionCurveStrength;
  nlohmann_json_j["motionProfileTrajectoryPeriod"] =
    nlohmann_json_t.motionProfileTrajectoryPeriod;
  nlohmann_json_j["feedbackNotContinuous"] =
    nlohmann_json_t.feedbackNotContinuous;
  nlohmann_json_j["remoteSensorClosedLoopDisableNeutralOnLOS"] =
    nlohmann_json_t.remoteSensorClosedLoopDisableNeutralOnLOS;
  nlohmann_json_j["clearPositionOnLimitF"] =
    nlohmann_json_t.clearPositionOnLimitF;
  nlohmann_json_j["clearPositionOnLimitR"] =
    nlohmann_json_t.clearPositionOnLimitR;
  nlohmann_json_j["clearPositionOnQuadIdx"] =
    nlohmann_json_t.clearPositionOnQuadIdx;
  nlohmann_json_j["limitSwitchDisableNeutralOnLOS"] =
    nlohmann_json_t.limitSwitchDisableNeutralOnLOS;
  nlohmann_json_j["softLimitDisableNeutralOnLOS"] =
    nlohmann_json_t.softLimitDisableNeutralOnLOS;
  nlohmann_json_j["pulseWidthPeriod_EdgesPerRot"] =
    nlohmann_json_t.pulseWidthPeriod_EdgesPerRot;
  nlohmann_json_j["pulseWidthPeriod_FilterWindowSz"] =
    nlohmann_json_t.pulseWidthPeriod_FilterWindowSz;
  nlohmann_json_j["trajectoryInterpolationEnable"] =
    nlohmann_json_t.trajectoryInterpolationEnable;
  nlohmann_json_j["primaryPID"] = nlohmann_json_t.primaryPID;
  nlohmann_json_j["auxiliaryPID"] = nlohmann_json_t.auxiliaryPID;
  nlohmann_json_j["forwardLimitSwitchSource"] =
    nlohmann_json_t.forwardLimitSwitchSource;
  nlohmann_json_j["reverseLimitSwitchSource"] =
    nlohmann_json_t.reverseLimitSwitchSource;
  nlohmann_json_j["forwardLimitSwitchDeviceID"] =
    nlohmann_json_t.forwardLimitSwitchDeviceID;
  nlohmann_json_j["reverseLimitSwitchDeviceID"] =
    nlohmann_json_t.reverseLimitSwitchDeviceID;
  nlohmann_json_j["forwardLimitSwitchNormal"] =
    nlohmann_json_t.forwardLimitSwitchNormal;
  nlohmann_json_j["reverseLimitSwitchNormal"] =
    nlohmann_json_t.reverseLimitSwitchNormal;
  nlohmann_json_j["sum0Term"] = nlohmann_json_t.sum0Term;
  nlohmann_json_j["sum1Term"] = nlohmann_json_t.sum1Term;
  nlohmann_json_j["diff0Term"] = nlohmann_json_t.diff0Term;
  nlohmann_json_j["diff1Term"] = nlohmann_json_t.diff1Term;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          BaseTalonConfiguration& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.customParam0), void>::from_json(
    nlohmann_json_j.at("customParam0"), nlohmann_json_t.customParam0);
  wpi::adl_serializer<decltype(nlohmann_json_t.customParam1), void>::from_json(
    nlohmann_json_j.at("customParam1"), nlohmann_json_t.customParam1);
  wpi::adl_serializer<decltype(nlohmann_json_t.enableOptimizations),
                      void>::from_json(nlohmann_json_j
                                         .at("enableOptimizations"),
                                       nlohmann_json_t.enableOptimizations);
  wpi::adl_serializer<decltype(nlohmann_json_t.openloopRamp), void>::from_json(
    nlohmann_json_j.at("openloopRamp"), nlohmann_json_t.openloopRamp);
  wpi::adl_serializer<decltype(nlohmann_json_t.closedloopRamp),
                      void>::from_json(nlohmann_json_j.at("closedloopRamp"),
                                       nlohmann_json_t.closedloopRamp);
  wpi::adl_serializer<decltype(nlohmann_json_t.peakOutputForward),
                      void>::from_json(nlohmann_json_j.at("peakOutputForward"),
                                       nlohmann_json_t.peakOutputForward);
  wpi::adl_serializer<decltype(nlohmann_json_t.peakOutputReverse),
                      void>::from_json(nlohmann_json_j.at("peakOutputReverse"),
                                       nlohmann_json_t.peakOutputReverse);
  wpi::adl_serializer<decltype(nlohmann_json_t.nominalOutputForward),
                      void>::from_json(nlohmann_json_j
                                         .at("nominalOutputForward"),
                                       nlohmann_json_t.nominalOutputForward);
  wpi::adl_serializer<decltype(nlohmann_json_t.nominalOutputReverse),
                      void>::from_json(nlohmann_json_j
                                         .at("nominalOutputReverse"),
                                       nlohmann_json_t.nominalOutputReverse);
  wpi::adl_serializer<decltype(nlohmann_json_t.neutralDeadband),
                      void>::from_json(nlohmann_json_j.at("neutralDeadband"),
                                       nlohmann_json_t.neutralDeadband);
  wpi::adl_serializer<decltype(nlohmann_json_t.voltageCompSaturation),
                      void>::from_json(nlohmann_json_j
                                         .at("voltageCompSaturation"),
                                       nlohmann_json_t.voltageCompSaturation);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.voltageMeasurementFilter),
    void>::from_json(nlohmann_json_j.at("voltageMeasurementFilter"),
                     nlohmann_json_t.voltageMeasurementFilter);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.velocityMeasurementPeriod),
    void>::from_json(nlohmann_json_j.at("velocityMeasurementPeriod"),
                     nlohmann_json_t.velocityMeasurementPeriod);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.velocityMeasurementWindow),
    void>::from_json(nlohmann_json_j.at("velocityMeasurementWindow"),
                     nlohmann_json_t.velocityMeasurementWindow);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.forwardSoftLimitThreshold),
    void>::from_json(nlohmann_json_j.at("forwardSoftLimitThreshold"),
                     nlohmann_json_t.forwardSoftLimitThreshold);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.reverseSoftLimitThreshold),
    void>::from_json(nlohmann_json_j.at("reverseSoftLimitThreshold"),
                     nlohmann_json_t.reverseSoftLimitThreshold);
  wpi::adl_serializer<decltype(nlohmann_json_t.forwardSoftLimitEnable),
                      void>::from_json(nlohmann_json_j
                                         .at("forwardSoftLimitEnable"),
                                       nlohmann_json_t.forwardSoftLimitEnable);
  wpi::adl_serializer<decltype(nlohmann_json_t.reverseSoftLimitEnable),
                      void>::from_json(nlohmann_json_j
                                         .at("reverseSoftLimitEnable"),
                                       nlohmann_json_t.reverseSoftLimitEnable);
  wpi::adl_serializer<decltype(nlohmann_json_t.slot0), void>::from_json(
    nlohmann_json_j.at("slot0"), nlohmann_json_t.slot0);
  wpi::adl_serializer<decltype(nlohmann_json_t.slot1), void>::from_json(
    nlohmann_json_j.at("slot1"), nlohmann_json_t.slot1);
  wpi::adl_serializer<decltype(nlohmann_json_t.slot2), void>::from_json(
    nlohmann_json_j.at("slot2"), nlohmann_json_t.slot2);
  wpi::adl_serializer<decltype(nlohmann_json_t.slot3), void>::from_json(
    nlohmann_json_j.at("slot3"), nlohmann_json_t.slot3);
  wpi::adl_serializer<decltype(nlohmann_json_t.auxPIDPolarity),
                      void>::from_json(nlohmann_json_j.at("auxPIDPolarity"),
                                       nlohmann_json_t.auxPIDPolarity);
  wpi::adl_serializer<decltype(nlohmann_json_t.remoteFilter0), void>::from_json(
    nlohmann_json_j.at("remoteFilter0"), nlohmann_json_t.remoteFilter0);
  wpi::adl_serializer<decltype(nlohmann_json_t.remoteFilter1), void>::from_json(
    nlohmann_json_j.at("remoteFilter1"), nlohmann_json_t.remoteFilter1);
  wpi::adl_serializer<decltype(nlohmann_json_t.motionCruiseVelocity),
                      void>::from_json(nlohmann_json_j
                                         .at("motionCruiseVelocity"),
                                       nlohmann_json_t.motionCruiseVelocity);
  wpi::adl_serializer<decltype(nlohmann_json_t.motionAcceleration),
                      void>::from_json(nlohmann_json_j.at("motionAcceleration"),
                                       nlohmann_json_t.motionAcceleration);
  wpi::adl_serializer<decltype(nlohmann_json_t.motionCurveStrength),
                      void>::from_json(nlohmann_json_j
                                         .at("motionCurveStrength"),
                                       nlohmann_json_t.motionCurveStrength);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.motionProfileTrajectoryPeriod),
    void>::from_json(nlohmann_json_j.at("motionProfileTrajectoryPeriod"),
                     nlohmann_json_t.motionProfileTrajectoryPeriod);
  wpi::adl_serializer<decltype(nlohmann_json_t.feedbackNotContinuous),
                      void>::from_json(nlohmann_json_j
                                         .at("feedbackNotContinuous"),
                                       nlohmann_json_t.feedbackNotContinuous);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.remoteSensorClosedLoopDisableNeutralOnLOS),
    void>::from_json(nlohmann_json_j
                       .at("remoteSensorClosedLoopDisableNeutralOnLOS"),
                     nlohmann_json_t.remoteSensorClosedLoopDisableNeutralOnLOS);
  wpi::adl_serializer<decltype(nlohmann_json_t.clearPositionOnLimitF),
                      void>::from_json(nlohmann_json_j
                                         .at("clearPositionOnLimitF"),
                                       nlohmann_json_t.clearPositionOnLimitF);
  wpi::adl_serializer<decltype(nlohmann_json_t.clearPositionOnLimitR),
                      void>::from_json(nlohmann_json_j
                                         .at("clearPositionOnLimitR"),
                                       nlohmann_json_t.clearPositionOnLimitR);
  wpi::adl_serializer<decltype(nlohmann_json_t.clearPositionOnQuadIdx),
                      void>::from_json(nlohmann_json_j
                                         .at("clearPositionOnQuadIdx"),
                                       nlohmann_json_t.clearPositionOnQuadIdx);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.limitSwitchDisableNeutralOnLOS),
    void>::from_json(nlohmann_json_j.at("limitSwitchDisableNeutralOnLOS"),
                     nlohmann_json_t.limitSwitchDisableNeutralOnLOS);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.softLimitDisableNeutralOnLOS),
    void>::from_json(nlohmann_json_j.at("softLimitDisableNeutralOnLOS"),
                     nlohmann_json_t.softLimitDisableNeutralOnLOS);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.pulseWidthPeriod_EdgesPerRot),
    void>::from_json(nlohmann_json_j.at("pulseWidthPeriod_EdgesPerRot"),
                     nlohmann_json_t.pulseWidthPeriod_EdgesPerRot);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.pulseWidthPeriod_FilterWindowSz),
    void>::from_json(nlohmann_json_j.at("pulseWidthPeriod_FilterWindowSz"),
                     nlohmann_json_t.pulseWidthPeriod_FilterWindowSz);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.trajectoryInterpolationEnable),
    void>::from_json(nlohmann_json_j.at("trajectoryInterpolationEnable"),
                     nlohmann_json_t.trajectoryInterpolationEnable);
  wpi::adl_serializer<decltype(nlohmann_json_t.primaryPID), void>::from_json(
    nlohmann_json_j.at("primaryPID"), nlohmann_json_t.primaryPID);
  wpi::adl_serializer<decltype(nlohmann_json_t.auxiliaryPID), void>::from_json(
    nlohmann_json_j.at("auxiliaryPID"), nlohmann_json_t.auxiliaryPID);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.forwardLimitSwitchSource),
    void>::from_json(nlohmann_json_j.at("forwardLimitSwitchSource"),
                     nlohmann_json_t.forwardLimitSwitchSource);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.reverseLimitSwitchSource),
    void>::from_json(nlohmann_json_j.at("reverseLimitSwitchSource"),
                     nlohmann_json_t.reverseLimitSwitchSource);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.forwardLimitSwitchDeviceID),
    void>::from_json(nlohmann_json_j.at("forwardLimitSwitchDeviceID"),
                     nlohmann_json_t.forwardLimitSwitchDeviceID);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.reverseLimitSwitchDeviceID),
    void>::from_json(nlohmann_json_j.at("reverseLimitSwitchDeviceID"),
                     nlohmann_json_t.reverseLimitSwitchDeviceID);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.forwardLimitSwitchNormal),
    void>::from_json(nlohmann_json_j.at("forwardLimitSwitchNormal"),
                     nlohmann_json_t.forwardLimitSwitchNormal);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.reverseLimitSwitchNormal),
    void>::from_json(nlohmann_json_j.at("reverseLimitSwitchNormal"),
                     nlohmann_json_t.reverseLimitSwitchNormal);
  wpi::adl_serializer<decltype(nlohmann_json_t.sum0Term), void>::from_json(
    nlohmann_json_j.at("sum0Term"), nlohmann_json_t.sum0Term);
  wpi::adl_serializer<decltype(nlohmann_json_t.sum1Term), void>::from_json(
    nlohmann_json_j.at("sum1Term"), nlohmann_json_t.sum1Term);
  wpi::adl_serializer<decltype(nlohmann_json_t.diff0Term), void>::from_json(
    nlohmann_json_j.at("diff0Term"), nlohmann_json_t.diff0Term);
  wpi::adl_serializer<decltype(nlohmann_json_t.diff1Term), void>::from_json(
    nlohmann_json_j.at("diff1Term"), nlohmann_json_t.diff1Term);
}

inline void
to_json(wpi::json& nlohmann_json_j,
        const TalonFXPIDSetConfiguration& nlohmann_json_t)
{
  nlohmann_json_j["selectedFeedbackCoefficient"] =
    nlohmann_json_t.selectedFeedbackCoefficient;
  nlohmann_json_j["selectedFeedbackSensor"] =
    nlohmann_json_t.selectedFeedbackSensor;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          TalonFXPIDSetConfiguration& nlohmann_json_t)
{
  wpi::adl_serializer<
    decltype(nlohmann_json_t.selectedFeedbackCoefficient),
    void>::from_json(nlohmann_json_j.at("selectedFeedbackCoefficient"),
                     nlohmann_json_t.selectedFeedbackCoefficient);
  wpi::adl_serializer<decltype(nlohmann_json_t.selectedFeedbackSensor),
                      void>::from_json(nlohmann_json_j
                                         .at("selectedFeedbackSensor"),
                                       nlohmann_json_t.selectedFeedbackSensor);
}

inline void
to_json(wpi::json& nlohmann_json_j, const TalonFXConfiguration& nlohmann_json_t)
{
  nlohmann_json_j["customParam0"] = nlohmann_json_t.customParam0;
  nlohmann_json_j["customParam1"] = nlohmann_json_t.customParam1;
  nlohmann_json_j["enableOptimizations"] = nlohmann_json_t.enableOptimizations;
  nlohmann_json_j["openloopRamp"] = nlohmann_json_t.openloopRamp;
  nlohmann_json_j["closedloopRamp"] = nlohmann_json_t.closedloopRamp;
  nlohmann_json_j["peakOutputForward"] = nlohmann_json_t.peakOutputForward;
  nlohmann_json_j["peakOutputReverse"] = nlohmann_json_t.peakOutputReverse;
  nlohmann_json_j["nominalOutputForward"] =
    nlohmann_json_t.nominalOutputForward;
  nlohmann_json_j["nominalOutputReverse"] =
    nlohmann_json_t.nominalOutputReverse;
  nlohmann_json_j["neutralDeadband"] = nlohmann_json_t.neutralDeadband;
  nlohmann_json_j["voltageCompSaturation"] =
    nlohmann_json_t.voltageCompSaturation;
  nlohmann_json_j["voltageMeasurementFilter"] =
    nlohmann_json_t.voltageMeasurementFilter;
  nlohmann_json_j["velocityMeasurementPeriod"] =
    nlohmann_json_t.velocityMeasurementPeriod;
  nlohmann_json_j["velocityMeasurementWindow"] =
    nlohmann_json_t.velocityMeasurementWindow;
  nlohmann_json_j["forwardSoftLimitThreshold"] =
    nlohmann_json_t.forwardSoftLimitThreshold;
  nlohmann_json_j["reverseSoftLimitThreshold"] =
    nlohmann_json_t.reverseSoftLimitThreshold;
  nlohmann_json_j["forwardSoftLimitEnable"] =
    nlohmann_json_t.forwardSoftLimitEnable;
  nlohmann_json_j["reverseSoftLimitEnable"] =
    nlohmann_json_t.reverseSoftLimitEnable;
  nlohmann_json_j["slot0"] = nlohmann_json_t.slot0;
  nlohmann_json_j["slot1"] = nlohmann_json_t.slot1;
  nlohmann_json_j["slot2"] = nlohmann_json_t.slot2;
  nlohmann_json_j["slot3"] = nlohmann_json_t.slot3;
  nlohmann_json_j["auxPIDPolarity"] = nlohmann_json_t.auxPIDPolarity;
  nlohmann_json_j["remoteFilter0"] = nlohmann_json_t.remoteFilter0;
  nlohmann_json_j["remoteFilter1"] = nlohmann_json_t.remoteFilter1;
  nlohmann_json_j["motionCruiseVelocity"] =
    nlohmann_json_t.motionCruiseVelocity;
  nlohmann_json_j["motionAcceleration"] = nlohmann_json_t.motionAcceleration;
  nlohmann_json_j["motionCurveStrength"] = nlohmann_json_t.motionCurveStrength;
  nlohmann_json_j["motionProfileTrajectoryPeriod"] =
    nlohmann_json_t.motionProfileTrajectoryPeriod;
  nlohmann_json_j["feedbackNotContinuous"] =
    nlohmann_json_t.feedbackNotContinuous;
  nlohmann_json_j["remoteSensorClosedLoopDisableNeutralOnLOS"] =
    nlohmann_json_t.remoteSensorClosedLoopDisableNeutralOnLOS;
  nlohmann_json_j["clearPositionOnLimitF"] =
    nlohmann_json_t.clearPositionOnLimitF;
  nlohmann_json_j["clearPositionOnLimitR"] =
    nlohmann_json_t.clearPositionOnLimitR;
  nlohmann_json_j["clearPositionOnQuadIdx"] =
    nlohmann_json_t.clearPositionOnQuadIdx;
  nlohmann_json_j["limitSwitchDisableNeutralOnLOS"] =
    nlohmann_json_t.limitSwitchDisableNeutralOnLOS;
  nlohmann_json_j["softLimitDisableNeutralOnLOS"] =
    nlohmann_json_t.softLimitDisableNeutralOnLOS;
  nlohmann_json_j["pulseWidthPeriod_EdgesPerRot"] =
    nlohmann_json_t.pulseWidthPeriod_EdgesPerRot;
  nlohmann_json_j["pulseWidthPeriod_FilterWindowSz"] =
    nlohmann_json_t.pulseWidthPeriod_FilterWindowSz;
  nlohmann_json_j["trajectoryInterpolationEnable"] =
    nlohmann_json_t.trajectoryInterpolationEnable;
  nlohmann_json_j["primaryPID"] = nlohmann_json_t.primaryPID;
  nlohmann_json_j["auxiliaryPID"] = nlohmann_json_t.auxiliaryPID;
  nlohmann_json_j["forwardLimitSwitchSource"] =
    nlohmann_json_t.forwardLimitSwitchSource;
  nlohmann_json_j["reverseLimitSwitchSource"] =
    nlohmann_json_t.reverseLimitSwitchSource;
  nlohmann_json_j["forwardLimitSwitchDeviceID"] =
    nlohmann_json_t.forwardLimitSwitchDeviceID;
  nlohmann_json_j["reverseLimitSwitchDeviceID"] =
    nlohmann_json_t.reverseLimitSwitchDeviceID;
  nlohmann_json_j["forwardLimitSwitchNormal"] =
    nlohmann_json_t.forwardLimitSwitchNormal;
  nlohmann_json_j["reverseLimitSwitchNormal"] =
    nlohmann_json_t.reverseLimitSwitchNormal;
  nlohmann_json_j["sum0Term"] = nlohmann_json_t.sum0Term;
  nlohmann_json_j["sum1Term"] = nlohmann_json_t.sum1Term;
  nlohmann_json_j["diff0Term"] = nlohmann_json_t.diff0Term;
  nlohmann_json_j["diff1Term"] = nlohmann_json_t.diff1Term;
  nlohmann_json_j["supplyCurrLimit"] = nlohmann_json_t.supplyCurrLimit;
  nlohmann_json_j["statorCurrLimit"] = nlohmann_json_t.statorCurrLimit;
  nlohmann_json_j["motorCommutation"] = nlohmann_json_t.motorCommutation;
  nlohmann_json_j["absoluteSensorRange"] = nlohmann_json_t.absoluteSensorRange;
  nlohmann_json_j["integratedSensorOffsetDegrees"] =
    nlohmann_json_t.integratedSensorOffsetDegrees;
  nlohmann_json_j["initializationStrategy"] =
    nlohmann_json_t.initializationStrategy;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          TalonFXConfiguration& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.customParam0), void>::from_json(
    nlohmann_json_j.at("customParam0"), nlohmann_json_t.customParam0);
  wpi::adl_serializer<decltype(nlohmann_json_t.customParam1), void>::from_json(
    nlohmann_json_j.at("customParam1"), nlohmann_json_t.customParam1);
  wpi::adl_serializer<decltype(nlohmann_json_t.enableOptimizations),
                      void>::from_json(nlohmann_json_j
                                         .at("enableOptimizations"),
                                       nlohmann_json_t.enableOptimizations);
  wpi::adl_serializer<decltype(nlohmann_json_t.openloopRamp), void>::from_json(
    nlohmann_json_j.at("openloopRamp"), nlohmann_json_t.openloopRamp);
  wpi::adl_serializer<decltype(nlohmann_json_t.closedloopRamp),
                      void>::from_json(nlohmann_json_j.at("closedloopRamp"),
                                       nlohmann_json_t.closedloopRamp);
  wpi::adl_serializer<decltype(nlohmann_json_t.peakOutputForward),
                      void>::from_json(nlohmann_json_j.at("peakOutputForward"),
                                       nlohmann_json_t.peakOutputForward);
  wpi::adl_serializer<decltype(nlohmann_json_t.peakOutputReverse),
                      void>::from_json(nlohmann_json_j.at("peakOutputReverse"),
                                       nlohmann_json_t.peakOutputReverse);
  wpi::adl_serializer<decltype(nlohmann_json_t.nominalOutputForward),
                      void>::from_json(nlohmann_json_j
                                         .at("nominalOutputForward"),
                                       nlohmann_json_t.nominalOutputForward);
  wpi::adl_serializer<decltype(nlohmann_json_t.nominalOutputReverse),
                      void>::from_json(nlohmann_json_j
                                         .at("nominalOutputReverse"),
                                       nlohmann_json_t.nominalOutputReverse);
  wpi::adl_serializer<decltype(nlohmann_json_t.neutralDeadband),
                      void>::from_json(nlohmann_json_j.at("neutralDeadband"),
                                       nlohmann_json_t.neutralDeadband);
  wpi::adl_serializer<decltype(nlohmann_json_t.voltageCompSaturation),
                      void>::from_json(nlohmann_json_j
                                         .at("voltageCompSaturation"),
                                       nlohmann_json_t.voltageCompSaturation);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.voltageMeasurementFilter),
    void>::from_json(nlohmann_json_j.at("voltageMeasurementFilter"),
                     nlohmann_json_t.voltageMeasurementFilter);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.velocityMeasurementPeriod),
    void>::from_json(nlohmann_json_j.at("velocityMeasurementPeriod"),
                     nlohmann_json_t.velocityMeasurementPeriod);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.velocityMeasurementWindow),
    void>::from_json(nlohmann_json_j.at("velocityMeasurementWindow"),
                     nlohmann_json_t.velocityMeasurementWindow);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.forwardSoftLimitThreshold),
    void>::from_json(nlohmann_json_j.at("forwardSoftLimitThreshold"),
                     nlohmann_json_t.forwardSoftLimitThreshold);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.reverseSoftLimitThreshold),
    void>::from_json(nlohmann_json_j.at("reverseSoftLimitThreshold"),
                     nlohmann_json_t.reverseSoftLimitThreshold);
  wpi::adl_serializer<decltype(nlohmann_json_t.forwardSoftLimitEnable),
                      void>::from_json(nlohmann_json_j
                                         .at("forwardSoftLimitEnable"),
                                       nlohmann_json_t.forwardSoftLimitEnable);
  wpi::adl_serializer<decltype(nlohmann_json_t.reverseSoftLimitEnable),
                      void>::from_json(nlohmann_json_j
                                         .at("reverseSoftLimitEnable"),
                                       nlohmann_json_t.reverseSoftLimitEnable);
  wpi::adl_serializer<decltype(nlohmann_json_t.slot0), void>::from_json(
    nlohmann_json_j.at("slot0"), nlohmann_json_t.slot0);
  wpi::adl_serializer<decltype(nlohmann_json_t.slot1), void>::from_json(
    nlohmann_json_j.at("slot1"), nlohmann_json_t.slot1);
  wpi::adl_serializer<decltype(nlohmann_json_t.slot2), void>::from_json(
    nlohmann_json_j.at("slot2"), nlohmann_json_t.slot2);
  wpi::adl_serializer<decltype(nlohmann_json_t.slot3), void>::from_json(
    nlohmann_json_j.at("slot3"), nlohmann_json_t.slot3);
  wpi::adl_serializer<decltype(nlohmann_json_t.auxPIDPolarity),
                      void>::from_json(nlohmann_json_j.at("auxPIDPolarity"),
                                       nlohmann_json_t.auxPIDPolarity);
  wpi::adl_serializer<decltype(nlohmann_json_t.remoteFilter0), void>::from_json(
    nlohmann_json_j.at("remoteFilter0"), nlohmann_json_t.remoteFilter0);
  wpi::adl_serializer<decltype(nlohmann_json_t.remoteFilter1), void>::from_json(
    nlohmann_json_j.at("remoteFilter1"), nlohmann_json_t.remoteFilter1);
  wpi::adl_serializer<decltype(nlohmann_json_t.motionCruiseVelocity),
                      void>::from_json(nlohmann_json_j
                                         .at("motionCruiseVelocity"),
                                       nlohmann_json_t.motionCruiseVelocity);
  wpi::adl_serializer<decltype(nlohmann_json_t.motionAcceleration),
                      void>::from_json(nlohmann_json_j.at("motionAcceleration"),
                                       nlohmann_json_t.motionAcceleration);
  wpi::adl_serializer<decltype(nlohmann_json_t.motionCurveStrength),
                      void>::from_json(nlohmann_json_j
                                         .at("motionCurveStrength"),
                                       nlohmann_json_t.motionCurveStrength);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.motionProfileTrajectoryPeriod),
    void>::from_json(nlohmann_json_j.at("motionProfileTrajectoryPeriod"),
                     nlohmann_json_t.motionProfileTrajectoryPeriod);
  wpi::adl_serializer<decltype(nlohmann_json_t.feedbackNotContinuous),
                      void>::from_json(nlohmann_json_j
                                         .at("feedbackNotContinuous"),
                                       nlohmann_json_t.feedbackNotContinuous);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.remoteSensorClosedLoopDisableNeutralOnLOS),
    void>::from_json(nlohmann_json_j
                       .at("remoteSensorClosedLoopDisableNeutralOnLOS"),
                     nlohmann_json_t.remoteSensorClosedLoopDisableNeutralOnLOS);
  wpi::adl_serializer<decltype(nlohmann_json_t.clearPositionOnLimitF),
                      void>::from_json(nlohmann_json_j
                                         .at("clearPositionOnLimitF"),
                                       nlohmann_json_t.clearPositionOnLimitF);
  wpi::adl_serializer<decltype(nlohmann_json_t.clearPositionOnLimitR),
                      void>::from_json(nlohmann_json_j
                                         .at("clearPositionOnLimitR"),
                                       nlohmann_json_t.clearPositionOnLimitR);
  wpi::adl_serializer<decltype(nlohmann_json_t.clearPositionOnQuadIdx),
                      void>::from_json(nlohmann_json_j
                                         .at("clearPositionOnQuadIdx"),
                                       nlohmann_json_t.clearPositionOnQuadIdx);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.limitSwitchDisableNeutralOnLOS),
    void>::from_json(nlohmann_json_j.at("limitSwitchDisableNeutralOnLOS"),
                     nlohmann_json_t.limitSwitchDisableNeutralOnLOS);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.softLimitDisableNeutralOnLOS),
    void>::from_json(nlohmann_json_j.at("softLimitDisableNeutralOnLOS"),
                     nlohmann_json_t.softLimitDisableNeutralOnLOS);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.pulseWidthPeriod_EdgesPerRot),
    void>::from_json(nlohmann_json_j.at("pulseWidthPeriod_EdgesPerRot"),
                     nlohmann_json_t.pulseWidthPeriod_EdgesPerRot);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.pulseWidthPeriod_FilterWindowSz),
    void>::from_json(nlohmann_json_j.at("pulseWidthPeriod_FilterWindowSz"),
                     nlohmann_json_t.pulseWidthPeriod_FilterWindowSz);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.trajectoryInterpolationEnable),
    void>::from_json(nlohmann_json_j.at("trajectoryInterpolationEnable"),
                     nlohmann_json_t.trajectoryInterpolationEnable);
  wpi::adl_serializer<decltype(nlohmann_json_t.primaryPID), void>::from_json(
    nlohmann_json_j.at("primaryPID"), nlohmann_json_t.primaryPID);
  wpi::adl_serializer<decltype(nlohmann_json_t.auxiliaryPID), void>::from_json(
    nlohmann_json_j.at("auxiliaryPID"), nlohmann_json_t.auxiliaryPID);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.forwardLimitSwitchSource),
    void>::from_json(nlohmann_json_j.at("forwardLimitSwitchSource"),
                     nlohmann_json_t.forwardLimitSwitchSource);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.reverseLimitSwitchSource),
    void>::from_json(nlohmann_json_j.at("reverseLimitSwitchSource"),
                     nlohmann_json_t.reverseLimitSwitchSource);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.forwardLimitSwitchDeviceID),
    void>::from_json(nlohmann_json_j.at("forwardLimitSwitchDeviceID"),
                     nlohmann_json_t.forwardLimitSwitchDeviceID);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.reverseLimitSwitchDeviceID),
    void>::from_json(nlohmann_json_j.at("reverseLimitSwitchDeviceID"),
                     nlohmann_json_t.reverseLimitSwitchDeviceID);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.forwardLimitSwitchNormal),
    void>::from_json(nlohmann_json_j.at("forwardLimitSwitchNormal"),
                     nlohmann_json_t.forwardLimitSwitchNormal);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.reverseLimitSwitchNormal),
    void>::from_json(nlohmann_json_j.at("reverseLimitSwitchNormal"),
                     nlohmann_json_t.reverseLimitSwitchNormal);
  wpi::adl_serializer<decltype(nlohmann_json_t.sum0Term), void>::from_json(
    nlohmann_json_j.at("sum0Term"), nlohmann_json_t.sum0Term);
  wpi::adl_serializer<decltype(nlohmann_json_t.sum1Term), void>::from_json(
    nlohmann_json_j.at("sum1Term"), nlohmann_json_t.sum1Term);
  wpi::adl_serializer<decltype(nlohmann_json_t.diff0Term), void>::from_json(
    nlohmann_json_j.at("diff0Term"), nlohmann_json_t.diff0Term);
  wpi::adl_serializer<decltype(nlohmann_json_t.diff1Term), void>::from_json(
    nlohmann_json_j.at("diff1Term"), nlohmann_json_t.diff1Term);
  wpi::adl_serializer<decltype(nlohmann_json_t.supplyCurrLimit),
                      void>::from_json(nlohmann_json_j.at("supplyCurrLimit"),
                                       nlohmann_json_t.supplyCurrLimit);
  wpi::adl_serializer<decltype(nlohmann_json_t.statorCurrLimit),
                      void>::from_json(nlohmann_json_j.at("statorCurrLimit"),
                                       nlohmann_json_t.statorCurrLimit);
  wpi::adl_serializer<decltype(nlohmann_json_t.motorCommutation),
                      void>::from_json(nlohmann_json_j.at("motorCommutation"),
                                       nlohmann_json_t.motorCommutation);
  wpi::adl_serializer<decltype(nlohmann_json_t.absoluteSensorRange),
                      void>::from_json(nlohmann_json_j
                                         .at("absoluteSensorRange"),
                                       nlohmann_json_t.absoluteSensorRange);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.integratedSensorOffsetDegrees),
    void>::from_json(nlohmann_json_j.at("integratedSensorOffsetDegrees"),
                     nlohmann_json_t.integratedSensorOffsetDegrees);
  wpi::adl_serializer<decltype(nlohmann_json_t.initializationStrategy),
                      void>::from_json(nlohmann_json_j
                                         .at("initializationStrategy"),
                                       nlohmann_json_t.initializationStrategy);
}

inline void
to_json(wpi::json& nlohmann_json_j,
        const TalonSRXPIDSetConfiguration& nlohmann_json_t)
{
  nlohmann_json_j["selectedFeedbackCoefficient"] =
    nlohmann_json_t.selectedFeedbackCoefficient;
  nlohmann_json_j["selectedFeedbackSensor"] =
    nlohmann_json_t.selectedFeedbackSensor;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          TalonSRXPIDSetConfiguration& nlohmann_json_t)
{
  wpi::adl_serializer<
    decltype(nlohmann_json_t.selectedFeedbackCoefficient),
    void>::from_json(nlohmann_json_j.at("selectedFeedbackCoefficient"),
                     nlohmann_json_t.selectedFeedbackCoefficient);
  wpi::adl_serializer<decltype(nlohmann_json_t.selectedFeedbackSensor),
                      void>::from_json(nlohmann_json_j
                                         .at("selectedFeedbackSensor"),
                                       nlohmann_json_t.selectedFeedbackSensor);
}

inline void
to_json(wpi::json& nlohmann_json_j,
        const TalonSRXConfiguration& nlohmann_json_t)
{
  nlohmann_json_j["customParam0"] = nlohmann_json_t.customParam0;
  nlohmann_json_j["customParam1"] = nlohmann_json_t.customParam1;
  nlohmann_json_j["enableOptimizations"] = nlohmann_json_t.enableOptimizations;
  nlohmann_json_j["openloopRamp"] = nlohmann_json_t.openloopRamp;
  nlohmann_json_j["closedloopRamp"] = nlohmann_json_t.closedloopRamp;
  nlohmann_json_j["peakOutputForward"] = nlohmann_json_t.peakOutputForward;
  nlohmann_json_j["peakOutputReverse"] = nlohmann_json_t.peakOutputReverse;
  nlohmann_json_j["nominalOutputForward"] =
    nlohmann_json_t.nominalOutputForward;
  nlohmann_json_j["nominalOutputReverse"] =
    nlohmann_json_t.nominalOutputReverse;
  nlohmann_json_j["neutralDeadband"] = nlohmann_json_t.neutralDeadband;
  nlohmann_json_j["voltageCompSaturation"] =
    nlohmann_json_t.voltageCompSaturation;
  nlohmann_json_j["voltageMeasurementFilter"] =
    nlohmann_json_t.voltageMeasurementFilter;
  nlohmann_json_j["velocityMeasurementPeriod"] =
    nlohmann_json_t.velocityMeasurementPeriod;
  nlohmann_json_j["velocityMeasurementWindow"] =
    nlohmann_json_t.velocityMeasurementWindow;
  nlohmann_json_j["forwardSoftLimitThreshold"] =
    nlohmann_json_t.forwardSoftLimitThreshold;
  nlohmann_json_j["reverseSoftLimitThreshold"] =
    nlohmann_json_t.reverseSoftLimitThreshold;
  nlohmann_json_j["forwardSoftLimitEnable"] =
    nlohmann_json_t.forwardSoftLimitEnable;
  nlohmann_json_j["reverseSoftLimitEnable"] =
    nlohmann_json_t.reverseSoftLimitEnable;
  nlohmann_json_j["slot0"] = nlohmann_json_t.slot0;
  nlohmann_json_j["slot1"] = nlohmann_json_t.slot1;
  nlohmann_json_j["slot2"] = nlohmann_json_t.slot2;
  nlohmann_json_j["slot3"] = nlohmann_json_t.slot3;
  nlohmann_json_j["auxPIDPolarity"] = nlohmann_json_t.auxPIDPolarity;
  nlohmann_json_j["remoteFilter0"] = nlohmann_json_t.remoteFilter0;
  nlohmann_json_j["remoteFilter1"] = nlohmann_json_t.remoteFilter1;
  nlohmann_json_j["motionCruiseVelocity"] =
    nlohmann_json_t.motionCruiseVelocity;
  nlohmann_json_j["motionAcceleration"] = nlohmann_json_t.motionAcceleration;
  nlohmann_json_j["motionCurveStrength"] = nlohmann_json_t.motionCurveStrength;
  nlohmann_json_j["motionProfileTrajectoryPeriod"] =
    nlohmann_json_t.motionProfileTrajectoryPeriod;
  nlohmann_json_j["feedbackNotContinuous"] =
    nlohmann_json_t.feedbackNotContinuous;
  nlohmann_json_j["remoteSensorClosedLoopDisableNeutralOnLOS"] =
    nlohmann_json_t.remoteSensorClosedLoopDisableNeutralOnLOS;
  nlohmann_json_j["clearPositionOnLimitF"] =
    nlohmann_json_t.clearPositionOnLimitF;
  nlohmann_json_j["clearPositionOnLimitR"] =
    nlohmann_json_t.clearPositionOnLimitR;
  nlohmann_json_j["clearPositionOnQuadIdx"] =
    nlohmann_json_t.clearPositionOnQuadIdx;
  nlohmann_json_j["limitSwitchDisableNeutralOnLOS"] =
    nlohmann_json_t.limitSwitchDisableNeutralOnLOS;
  nlohmann_json_j["softLimitDisableNeutralOnLOS"] =
    nlohmann_json_t.softLimitDisableNeutralOnLOS;
  nlohmann_json_j["pulseWidthPeriod_EdgesPerRot"] =
    nlohmann_json_t.pulseWidthPeriod_EdgesPerRot;
  nlohmann_json_j["pulseWidthPeriod_FilterWindowSz"] =
    nlohmann_json_t.pulseWidthPeriod_FilterWindowSz;
  nlohmann_json_j["trajectoryInterpolationEnable"] =
    nlohmann_json_t.trajectoryInterpolationEnable;
  nlohmann_json_j["primaryPID"] = nlohmann_json_t.primaryPID;
  nlohmann_json_j["auxiliaryPID"] = nlohmann_json_t.auxiliaryPID;
  nlohmann_json_j["forwardLimitSwitchSource"] =
    nlohmann_json_t.forwardLimitSwitchSource;
  nlohmann_json_j["reverseLimitSwitchSource"] =
    nlohmann_json_t.reverseLimitSwitchSource;
  nlohmann_json_j["forwardLimitSwitchDeviceID"] =
    nlohmann_json_t.forwardLimitSwitchDeviceID;
  nlohmann_json_j["reverseLimitSwitchDeviceID"] =
    nlohmann_json_t.reverseLimitSwitchDeviceID;
  nlohmann_json_j["forwardLimitSwitchNormal"] =
    nlohmann_json_t.forwardLimitSwitchNormal;
  nlohmann_json_j["reverseLimitSwitchNormal"] =
    nlohmann_json_t.reverseLimitSwitchNormal;
  nlohmann_json_j["sum0Term"] = nlohmann_json_t.sum0Term;
  nlohmann_json_j["sum1Term"] = nlohmann_json_t.sum1Term;
  nlohmann_json_j["diff0Term"] = nlohmann_json_t.diff0Term;
  nlohmann_json_j["diff1Term"] = nlohmann_json_t.diff1Term;
  nlohmann_json_j["peakCurrentLimit"] = nlohmann_json_t.peakCurrentLimit;
  nlohmann_json_j["peakCurrentDuration"] = nlohmann_json_t.peakCurrentDuration;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          TalonSRXConfiguration& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.customParam0), void>::from_json(
    nlohmann_json_j.at("customParam0"), nlohmann_json_t.customParam0);
  wpi::adl_serializer<decltype(nlohmann_json_t.customParam1), void>::from_json(
    nlohmann_json_j.at("customParam1"), nlohmann_json_t.customParam1);
  wpi::adl_serializer<decltype(nlohmann_json_t.enableOptimizations),
                      void>::from_json(nlohmann_json_j
                                         .at("enableOptimizations"),
                                       nlohmann_json_t.enableOptimizations);
  wpi::adl_serializer<decltype(nlohmann_json_t.openloopRamp), void>::from_json(
    nlohmann_json_j.at("openloopRamp"), nlohmann_json_t.openloopRamp);
  wpi::adl_serializer<decltype(nlohmann_json_t.closedloopRamp),
                      void>::from_json(nlohmann_json_j.at("closedloopRamp"),
                                       nlohmann_json_t.closedloopRamp);
  wpi::adl_serializer<decltype(nlohmann_json_t.peakOutputForward),
                      void>::from_json(nlohmann_json_j.at("peakOutputForward"),
                                       nlohmann_json_t.peakOutputForward);
  wpi::adl_serializer<decltype(nlohmann_json_t.peakOutputReverse),
                      void>::from_json(nlohmann_json_j.at("peakOutputReverse"),
                                       nlohmann_json_t.peakOutputReverse);
  wpi::adl_serializer<decltype(nlohmann_json_t.nominalOutputForward),
                      void>::from_json(nlohmann_json_j
                                         .at("nominalOutputForward"),
                                       nlohmann_json_t.nominalOutputForward);
  wpi::adl_serializer<decltype(nlohmann_json_t.nominalOutputReverse),
                      void>::from_json(nlohmann_json_j
                                         .at("nominalOutputReverse"),
                                       nlohmann_json_t.nominalOutputReverse);
  wpi::adl_serializer<decltype(nlohmann_json_t.neutralDeadband),
                      void>::from_json(nlohmann_json_j.at("neutralDeadband"),
                                       nlohmann_json_t.neutralDeadband);
  wpi::adl_serializer<decltype(nlohmann_json_t.voltageCompSaturation),
                      void>::from_json(nlohmann_json_j
                                         .at("voltageCompSaturation"),
                                       nlohmann_json_t.voltageCompSaturation);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.voltageMeasurementFilter),
    void>::from_json(nlohmann_json_j.at("voltageMeasurementFilter"),
                     nlohmann_json_t.voltageMeasurementFilter);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.velocityMeasurementPeriod),
    void>::from_json(nlohmann_json_j.at("velocityMeasurementPeriod"),
                     nlohmann_json_t.velocityMeasurementPeriod);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.velocityMeasurementWindow),
    void>::from_json(nlohmann_json_j.at("velocityMeasurementWindow"),
                     nlohmann_json_t.velocityMeasurementWindow);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.forwardSoftLimitThreshold),
    void>::from_json(nlohmann_json_j.at("forwardSoftLimitThreshold"),
                     nlohmann_json_t.forwardSoftLimitThreshold);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.reverseSoftLimitThreshold),
    void>::from_json(nlohmann_json_j.at("reverseSoftLimitThreshold"),
                     nlohmann_json_t.reverseSoftLimitThreshold);
  wpi::adl_serializer<decltype(nlohmann_json_t.forwardSoftLimitEnable),
                      void>::from_json(nlohmann_json_j
                                         .at("forwardSoftLimitEnable"),
                                       nlohmann_json_t.forwardSoftLimitEnable);
  wpi::adl_serializer<decltype(nlohmann_json_t.reverseSoftLimitEnable),
                      void>::from_json(nlohmann_json_j
                                         .at("reverseSoftLimitEnable"),
                                       nlohmann_json_t.reverseSoftLimitEnable);
  wpi::adl_serializer<decltype(nlohmann_json_t.slot0), void>::from_json(
    nlohmann_json_j.at("slot0"), nlohmann_json_t.slot0);
  wpi::adl_serializer<decltype(nlohmann_json_t.slot1), void>::from_json(
    nlohmann_json_j.at("slot1"), nlohmann_json_t.slot1);
  wpi::adl_serializer<decltype(nlohmann_json_t.slot2), void>::from_json(
    nlohmann_json_j.at("slot2"), nlohmann_json_t.slot2);
  wpi::adl_serializer<decltype(nlohmann_json_t.slot3), void>::from_json(
    nlohmann_json_j.at("slot3"), nlohmann_json_t.slot3);
  wpi::adl_serializer<decltype(nlohmann_json_t.auxPIDPolarity),
                      void>::from_json(nlohmann_json_j.at("auxPIDPolarity"),
                                       nlohmann_json_t.auxPIDPolarity);
  wpi::adl_serializer<decltype(nlohmann_json_t.remoteFilter0), void>::from_json(
    nlohmann_json_j.at("remoteFilter0"), nlohmann_json_t.remoteFilter0);
  wpi::adl_serializer<decltype(nlohmann_json_t.remoteFilter1), void>::from_json(
    nlohmann_json_j.at("remoteFilter1"), nlohmann_json_t.remoteFilter1);
  wpi::adl_serializer<decltype(nlohmann_json_t.motionCruiseVelocity),
                      void>::from_json(nlohmann_json_j
                                         .at("motionCruiseVelocity"),
                                       nlohmann_json_t.motionCruiseVelocity);
  wpi::adl_serializer<decltype(nlohmann_json_t.motionAcceleration),
                      void>::from_json(nlohmann_json_j.at("motionAcceleration"),
                                       nlohmann_json_t.motionAcceleration);
  wpi::adl_serializer<decltype(nlohmann_json_t.motionCurveStrength),
                      void>::from_json(nlohmann_json_j
                                         .at("motionCurveStrength"),
                                       nlohmann_json_t.motionCurveStrength);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.motionProfileTrajectoryPeriod),
    void>::from_json(nlohmann_json_j.at("motionProfileTrajectoryPeriod"),
                     nlohmann_json_t.motionProfileTrajectoryPeriod);
  wpi::adl_serializer<decltype(nlohmann_json_t.feedbackNotContinuous),
                      void>::from_json(nlohmann_json_j
                                         .at("feedbackNotContinuous"),
                                       nlohmann_json_t.feedbackNotContinuous);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.remoteSensorClosedLoopDisableNeutralOnLOS),
    void>::from_json(nlohmann_json_j
                       .at("remoteSensorClosedLoopDisableNeutralOnLOS"),
                     nlohmann_json_t.remoteSensorClosedLoopDisableNeutralOnLOS);
  wpi::adl_serializer<decltype(nlohmann_json_t.clearPositionOnLimitF),
                      void>::from_json(nlohmann_json_j
                                         .at("clearPositionOnLimitF"),
                                       nlohmann_json_t.clearPositionOnLimitF);
  wpi::adl_serializer<decltype(nlohmann_json_t.clearPositionOnLimitR),
                      void>::from_json(nlohmann_json_j
                                         .at("clearPositionOnLimitR"),
                                       nlohmann_json_t.clearPositionOnLimitR);
  wpi::adl_serializer<decltype(nlohmann_json_t.clearPositionOnQuadIdx),
                      void>::from_json(nlohmann_json_j
                                         .at("clearPositionOnQuadIdx"),
                                       nlohmann_json_t.clearPositionOnQuadIdx);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.limitSwitchDisableNeutralOnLOS),
    void>::from_json(nlohmann_json_j.at("limitSwitchDisableNeutralOnLOS"),
                     nlohmann_json_t.limitSwitchDisableNeutralOnLOS);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.softLimitDisableNeutralOnLOS),
    void>::from_json(nlohmann_json_j.at("softLimitDisableNeutralOnLOS"),
                     nlohmann_json_t.softLimitDisableNeutralOnLOS);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.pulseWidthPeriod_EdgesPerRot),
    void>::from_json(nlohmann_json_j.at("pulseWidthPeriod_EdgesPerRot"),
                     nlohmann_json_t.pulseWidthPeriod_EdgesPerRot);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.pulseWidthPeriod_FilterWindowSz),
    void>::from_json(nlohmann_json_j.at("pulseWidthPeriod_FilterWindowSz"),
                     nlohmann_json_t.pulseWidthPeriod_FilterWindowSz);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.trajectoryInterpolationEnable),
    void>::from_json(nlohmann_json_j.at("trajectoryInterpolationEnable"),
                     nlohmann_json_t.trajectoryInterpolationEnable);
  wpi::adl_serializer<decltype(nlohmann_json_t.primaryPID), void>::from_json(
    nlohmann_json_j.at("primaryPID"), nlohmann_json_t.primaryPID);
  wpi::adl_serializer<decltype(nlohmann_json_t.auxiliaryPID), void>::from_json(
    nlohmann_json_j.at("auxiliaryPID"), nlohmann_json_t.auxiliaryPID);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.forwardLimitSwitchSource),
    void>::from_json(nlohmann_json_j.at("forwardLimitSwitchSource"),
                     nlohmann_json_t.forwardLimitSwitchSource);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.reverseLimitSwitchSource),
    void>::from_json(nlohmann_json_j.at("reverseLimitSwitchSource"),
                     nlohmann_json_t.reverseLimitSwitchSource);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.forwardLimitSwitchDeviceID),
    void>::from_json(nlohmann_json_j.at("forwardLimitSwitchDeviceID"),
                     nlohmann_json_t.forwardLimitSwitchDeviceID);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.reverseLimitSwitchDeviceID),
    void>::from_json(nlohmann_json_j.at("reverseLimitSwitchDeviceID"),
                     nlohmann_json_t.reverseLimitSwitchDeviceID);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.forwardLimitSwitchNormal),
    void>::from_json(nlohmann_json_j.at("forwardLimitSwitchNormal"),
                     nlohmann_json_t.forwardLimitSwitchNormal);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.reverseLimitSwitchNormal),
    void>::from_json(nlohmann_json_j.at("reverseLimitSwitchNormal"),
                     nlohmann_json_t.reverseLimitSwitchNormal);
  wpi::adl_serializer<decltype(nlohmann_json_t.sum0Term), void>::from_json(
    nlohmann_json_j.at("sum0Term"), nlohmann_json_t.sum0Term);
  wpi::adl_serializer<decltype(nlohmann_json_t.sum1Term), void>::from_json(
    nlohmann_json_j.at("sum1Term"), nlohmann_json_t.sum1Term);
  wpi::adl_serializer<decltype(nlohmann_json_t.diff0Term), void>::from_json(
    nlohmann_json_j.at("diff0Term"), nlohmann_json_t.diff0Term);
  wpi::adl_serializer<decltype(nlohmann_json_t.diff1Term), void>::from_json(
    nlohmann_json_j.at("diff1Term"), nlohmann_json_t.diff1Term);
  wpi::adl_serializer<decltype(nlohmann_json_t.peakCurrentLimit),
                      void>::from_json(nlohmann_json_j.at("peakCurrentLimit"),
                                       nlohmann_json_t.peakCurrentLimit);
  wpi::adl_serializer<decltype(nlohmann_json_t.peakCurrentDuration),
                      void>::from_json(nlohmann_json_j
                                         .at("peakCurrentDuration"),
                                       nlohmann_json_t.peakCurrentDuration);
}

inline void
to_json(wpi::json& nlohmann_json_j,
        const VictorSPXPIDSetConfiguration& nlohmann_json_t)
{
  nlohmann_json_j["selectedFeedbackCoefficient"] =
    nlohmann_json_t.selectedFeedbackCoefficient;
  nlohmann_json_j["selectedFeedbackSensor"] =
    nlohmann_json_t.selectedFeedbackSensor;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          VictorSPXPIDSetConfiguration& nlohmann_json_t)
{
  wpi::adl_serializer<
    decltype(nlohmann_json_t.selectedFeedbackCoefficient),
    void>::from_json(nlohmann_json_j.at("selectedFeedbackCoefficient"),
                     nlohmann_json_t.selectedFeedbackCoefficient);
  wpi::adl_serializer<decltype(nlohmann_json_t.selectedFeedbackSensor),
                      void>::from_json(nlohmann_json_j
                                         .at("selectedFeedbackSensor"),
                                       nlohmann_json_t.selectedFeedbackSensor);
}

inline void
to_json(wpi::json& nlohmann_json_j,
        const VictorSPXConfiguration& nlohmann_json_t)
{
  nlohmann_json_j["customParam0"] = nlohmann_json_t.customParam0;
  nlohmann_json_j["customParam1"] = nlohmann_json_t.customParam1;
  nlohmann_json_j["enableOptimizations"] = nlohmann_json_t.enableOptimizations;
  nlohmann_json_j["openloopRamp"] = nlohmann_json_t.openloopRamp;
  nlohmann_json_j["closedloopRamp"] = nlohmann_json_t.closedloopRamp;
  nlohmann_json_j["peakOutputForward"] = nlohmann_json_t.peakOutputForward;
  nlohmann_json_j["peakOutputReverse"] = nlohmann_json_t.peakOutputReverse;
  nlohmann_json_j["nominalOutputForward"] =
    nlohmann_json_t.nominalOutputForward;
  nlohmann_json_j["nominalOutputReverse"] =
    nlohmann_json_t.nominalOutputReverse;
  nlohmann_json_j["neutralDeadband"] = nlohmann_json_t.neutralDeadband;
  nlohmann_json_j["voltageCompSaturation"] =
    nlohmann_json_t.voltageCompSaturation;
  nlohmann_json_j["voltageMeasurementFilter"] =
    nlohmann_json_t.voltageMeasurementFilter;
  nlohmann_json_j["velocityMeasurementPeriod"] =
    nlohmann_json_t.velocityMeasurementPeriod;
  nlohmann_json_j["velocityMeasurementWindow"] =
    nlohmann_json_t.velocityMeasurementWindow;
  nlohmann_json_j["forwardSoftLimitThreshold"] =
    nlohmann_json_t.forwardSoftLimitThreshold;
  nlohmann_json_j["reverseSoftLimitThreshold"] =
    nlohmann_json_t.reverseSoftLimitThreshold;
  nlohmann_json_j["forwardSoftLimitEnable"] =
    nlohmann_json_t.forwardSoftLimitEnable;
  nlohmann_json_j["reverseSoftLimitEnable"] =
    nlohmann_json_t.reverseSoftLimitEnable;
  nlohmann_json_j["slot0"] = nlohmann_json_t.slot0;
  nlohmann_json_j["slot1"] = nlohmann_json_t.slot1;
  nlohmann_json_j["slot2"] = nlohmann_json_t.slot2;
  nlohmann_json_j["slot3"] = nlohmann_json_t.slot3;
  nlohmann_json_j["auxPIDPolarity"] = nlohmann_json_t.auxPIDPolarity;
  nlohmann_json_j["remoteFilter0"] = nlohmann_json_t.remoteFilter0;
  nlohmann_json_j["remoteFilter1"] = nlohmann_json_t.remoteFilter1;
  nlohmann_json_j["motionCruiseVelocity"] =
    nlohmann_json_t.motionCruiseVelocity;
  nlohmann_json_j["motionAcceleration"] = nlohmann_json_t.motionAcceleration;
  nlohmann_json_j["motionCurveStrength"] = nlohmann_json_t.motionCurveStrength;
  nlohmann_json_j["motionProfileTrajectoryPeriod"] =
    nlohmann_json_t.motionProfileTrajectoryPeriod;
  nlohmann_json_j["feedbackNotContinuous"] =
    nlohmann_json_t.feedbackNotContinuous;
  nlohmann_json_j["remoteSensorClosedLoopDisableNeutralOnLOS"] =
    nlohmann_json_t.remoteSensorClosedLoopDisableNeutralOnLOS;
  nlohmann_json_j["clearPositionOnLimitF"] =
    nlohmann_json_t.clearPositionOnLimitF;
  nlohmann_json_j["clearPositionOnLimitR"] =
    nlohmann_json_t.clearPositionOnLimitR;
  nlohmann_json_j["clearPositionOnQuadIdx"] =
    nlohmann_json_t.clearPositionOnQuadIdx;
  nlohmann_json_j["limitSwitchDisableNeutralOnLOS"] =
    nlohmann_json_t.limitSwitchDisableNeutralOnLOS;
  nlohmann_json_j["softLimitDisableNeutralOnLOS"] =
    nlohmann_json_t.softLimitDisableNeutralOnLOS;
  nlohmann_json_j["pulseWidthPeriod_EdgesPerRot"] =
    nlohmann_json_t.pulseWidthPeriod_EdgesPerRot;
  nlohmann_json_j["pulseWidthPeriod_FilterWindowSz"] =
    nlohmann_json_t.pulseWidthPeriod_FilterWindowSz;
  nlohmann_json_j["trajectoryInterpolationEnable"] =
    nlohmann_json_t.trajectoryInterpolationEnable;
  nlohmann_json_j["primaryPID"] = nlohmann_json_t.primaryPID;
  nlohmann_json_j["auxiliaryPID"] = nlohmann_json_t.auxiliaryPID;
  nlohmann_json_j["forwardLimitSwitchSource"] =
    nlohmann_json_t.forwardLimitSwitchSource;
  nlohmann_json_j["reverseLimitSwitchSource"] =
    nlohmann_json_t.reverseLimitSwitchSource;
  nlohmann_json_j["forwardLimitSwitchDeviceID"] =
    nlohmann_json_t.forwardLimitSwitchDeviceID;
  nlohmann_json_j["reverseLimitSwitchDeviceID"] =
    nlohmann_json_t.reverseLimitSwitchDeviceID;
  nlohmann_json_j["forwardLimitSwitchNormal"] =
    nlohmann_json_t.forwardLimitSwitchNormal;
  nlohmann_json_j["reverseLimitSwitchNormal"] =
    nlohmann_json_t.reverseLimitSwitchNormal;
  nlohmann_json_j["sum0Term"] = nlohmann_json_t.sum0Term;
  nlohmann_json_j["sum1Term"] = nlohmann_json_t.sum1Term;
  nlohmann_json_j["diff0Term"] = nlohmann_json_t.diff0Term;
  nlohmann_json_j["diff1Term"] = nlohmann_json_t.diff1Term;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          VictorSPXConfiguration& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.customParam0), void>::from_json(
    nlohmann_json_j.at("customParam0"), nlohmann_json_t.customParam0);
  wpi::adl_serializer<decltype(nlohmann_json_t.customParam1), void>::from_json(
    nlohmann_json_j.at("customParam1"), nlohmann_json_t.customParam1);
  wpi::adl_serializer<decltype(nlohmann_json_t.enableOptimizations),
                      void>::from_json(nlohmann_json_j
                                         .at("enableOptimizations"),
                                       nlohmann_json_t.enableOptimizations);
  wpi::adl_serializer<decltype(nlohmann_json_t.openloopRamp), void>::from_json(
    nlohmann_json_j.at("openloopRamp"), nlohmann_json_t.openloopRamp);
  wpi::adl_serializer<decltype(nlohmann_json_t.closedloopRamp),
                      void>::from_json(nlohmann_json_j.at("closedloopRamp"),
                                       nlohmann_json_t.closedloopRamp);
  wpi::adl_serializer<decltype(nlohmann_json_t.peakOutputForward),
                      void>::from_json(nlohmann_json_j.at("peakOutputForward"),
                                       nlohmann_json_t.peakOutputForward);
  wpi::adl_serializer<decltype(nlohmann_json_t.peakOutputReverse),
                      void>::from_json(nlohmann_json_j.at("peakOutputReverse"),
                                       nlohmann_json_t.peakOutputReverse);
  wpi::adl_serializer<decltype(nlohmann_json_t.nominalOutputForward),
                      void>::from_json(nlohmann_json_j
                                         .at("nominalOutputForward"),
                                       nlohmann_json_t.nominalOutputForward);
  wpi::adl_serializer<decltype(nlohmann_json_t.nominalOutputReverse),
                      void>::from_json(nlohmann_json_j
                                         .at("nominalOutputReverse"),
                                       nlohmann_json_t.nominalOutputReverse);
  wpi::adl_serializer<decltype(nlohmann_json_t.neutralDeadband),
                      void>::from_json(nlohmann_json_j.at("neutralDeadband"),
                                       nlohmann_json_t.neutralDeadband);
  wpi::adl_serializer<decltype(nlohmann_json_t.voltageCompSaturation),
                      void>::from_json(nlohmann_json_j
                                         .at("voltageCompSaturation"),
                                       nlohmann_json_t.voltageCompSaturation);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.voltageMeasurementFilter),
    void>::from_json(nlohmann_json_j.at("voltageMeasurementFilter"),
                     nlohmann_json_t.voltageMeasurementFilter);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.velocityMeasurementPeriod),
    void>::from_json(nlohmann_json_j.at("velocityMeasurementPeriod"),
                     nlohmann_json_t.velocityMeasurementPeriod);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.velocityMeasurementWindow),
    void>::from_json(nlohmann_json_j.at("velocityMeasurementWindow"),
                     nlohmann_json_t.velocityMeasurementWindow);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.forwardSoftLimitThreshold),
    void>::from_json(nlohmann_json_j.at("forwardSoftLimitThreshold"),
                     nlohmann_json_t.forwardSoftLimitThreshold);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.reverseSoftLimitThreshold),
    void>::from_json(nlohmann_json_j.at("reverseSoftLimitThreshold"),
                     nlohmann_json_t.reverseSoftLimitThreshold);
  wpi::adl_serializer<decltype(nlohmann_json_t.forwardSoftLimitEnable),
                      void>::from_json(nlohmann_json_j
                                         .at("forwardSoftLimitEnable"),
                                       nlohmann_json_t.forwardSoftLimitEnable);
  wpi::adl_serializer<decltype(nlohmann_json_t.reverseSoftLimitEnable),
                      void>::from_json(nlohmann_json_j
                                         .at("reverseSoftLimitEnable"),
                                       nlohmann_json_t.reverseSoftLimitEnable);
  wpi::adl_serializer<decltype(nlohmann_json_t.slot0), void>::from_json(
    nlohmann_json_j.at("slot0"), nlohmann_json_t.slot0);
  wpi::adl_serializer<decltype(nlohmann_json_t.slot1), void>::from_json(
    nlohmann_json_j.at("slot1"), nlohmann_json_t.slot1);
  wpi::adl_serializer<decltype(nlohmann_json_t.slot2), void>::from_json(
    nlohmann_json_j.at("slot2"), nlohmann_json_t.slot2);
  wpi::adl_serializer<decltype(nlohmann_json_t.slot3), void>::from_json(
    nlohmann_json_j.at("slot3"), nlohmann_json_t.slot3);
  wpi::adl_serializer<decltype(nlohmann_json_t.auxPIDPolarity),
                      void>::from_json(nlohmann_json_j.at("auxPIDPolarity"),
                                       nlohmann_json_t.auxPIDPolarity);
  wpi::adl_serializer<decltype(nlohmann_json_t.remoteFilter0), void>::from_json(
    nlohmann_json_j.at("remoteFilter0"), nlohmann_json_t.remoteFilter0);
  wpi::adl_serializer<decltype(nlohmann_json_t.remoteFilter1), void>::from_json(
    nlohmann_json_j.at("remoteFilter1"), nlohmann_json_t.remoteFilter1);
  wpi::adl_serializer<decltype(nlohmann_json_t.motionCruiseVelocity),
                      void>::from_json(nlohmann_json_j
                                         .at("motionCruiseVelocity"),
                                       nlohmann_json_t.motionCruiseVelocity);
  wpi::adl_serializer<decltype(nlohmann_json_t.motionAcceleration),
                      void>::from_json(nlohmann_json_j.at("motionAcceleration"),
                                       nlohmann_json_t.motionAcceleration);
  wpi::adl_serializer<decltype(nlohmann_json_t.motionCurveStrength),
                      void>::from_json(nlohmann_json_j
                                         .at("motionCurveStrength"),
                                       nlohmann_json_t.motionCurveStrength);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.motionProfileTrajectoryPeriod),
    void>::from_json(nlohmann_json_j.at("motionProfileTrajectoryPeriod"),
                     nlohmann_json_t.motionProfileTrajectoryPeriod);
  wpi::adl_serializer<decltype(nlohmann_json_t.feedbackNotContinuous),
                      void>::from_json(nlohmann_json_j
                                         .at("feedbackNotContinuous"),
                                       nlohmann_json_t.feedbackNotContinuous);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.remoteSensorClosedLoopDisableNeutralOnLOS),
    void>::from_json(nlohmann_json_j
                       .at("remoteSensorClosedLoopDisableNeutralOnLOS"),
                     nlohmann_json_t.remoteSensorClosedLoopDisableNeutralOnLOS);
  wpi::adl_serializer<decltype(nlohmann_json_t.clearPositionOnLimitF),
                      void>::from_json(nlohmann_json_j
                                         .at("clearPositionOnLimitF"),
                                       nlohmann_json_t.clearPositionOnLimitF);
  wpi::adl_serializer<decltype(nlohmann_json_t.clearPositionOnLimitR),
                      void>::from_json(nlohmann_json_j
                                         .at("clearPositionOnLimitR"),
                                       nlohmann_json_t.clearPositionOnLimitR);
  wpi::adl_serializer<decltype(nlohmann_json_t.clearPositionOnQuadIdx),
                      void>::from_json(nlohmann_json_j
                                         .at("clearPositionOnQuadIdx"),
                                       nlohmann_json_t.clearPositionOnQuadIdx);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.limitSwitchDisableNeutralOnLOS),
    void>::from_json(nlohmann_json_j.at("limitSwitchDisableNeutralOnLOS"),
                     nlohmann_json_t.limitSwitchDisableNeutralOnLOS);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.softLimitDisableNeutralOnLOS),
    void>::from_json(nlohmann_json_j.at("softLimitDisableNeutralOnLOS"),
                     nlohmann_json_t.softLimitDisableNeutralOnLOS);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.pulseWidthPeriod_EdgesPerRot),
    void>::from_json(nlohmann_json_j.at("pulseWidthPeriod_EdgesPerRot"),
                     nlohmann_json_t.pulseWidthPeriod_EdgesPerRot);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.pulseWidthPeriod_FilterWindowSz),
    void>::from_json(nlohmann_json_j.at("pulseWidthPeriod_FilterWindowSz"),
                     nlohmann_json_t.pulseWidthPeriod_FilterWindowSz);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.trajectoryInterpolationEnable),
    void>::from_json(nlohmann_json_j.at("trajectoryInterpolationEnable"),
                     nlohmann_json_t.trajectoryInterpolationEnable);
  wpi::adl_serializer<decltype(nlohmann_json_t.primaryPID), void>::from_json(
    nlohmann_json_j.at("primaryPID"), nlohmann_json_t.primaryPID);
  wpi::adl_serializer<decltype(nlohmann_json_t.auxiliaryPID), void>::from_json(
    nlohmann_json_j.at("auxiliaryPID"), nlohmann_json_t.auxiliaryPID);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.forwardLimitSwitchSource),
    void>::from_json(nlohmann_json_j.at("forwardLimitSwitchSource"),
                     nlohmann_json_t.forwardLimitSwitchSource);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.reverseLimitSwitchSource),
    void>::from_json(nlohmann_json_j.at("reverseLimitSwitchSource"),
                     nlohmann_json_t.reverseLimitSwitchSource);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.forwardLimitSwitchDeviceID),
    void>::from_json(nlohmann_json_j.at("forwardLimitSwitchDeviceID"),
                     nlohmann_json_t.forwardLimitSwitchDeviceID);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.reverseLimitSwitchDeviceID),
    void>::from_json(nlohmann_json_j.at("reverseLimitSwitchDeviceID"),
                     nlohmann_json_t.reverseLimitSwitchDeviceID);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.forwardLimitSwitchNormal),
    void>::from_json(nlohmann_json_j.at("forwardLimitSwitchNormal"),
                     nlohmann_json_t.forwardLimitSwitchNormal);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.reverseLimitSwitchNormal),
    void>::from_json(nlohmann_json_j.at("reverseLimitSwitchNormal"),
                     nlohmann_json_t.reverseLimitSwitchNormal);
  wpi::adl_serializer<decltype(nlohmann_json_t.sum0Term), void>::from_json(
    nlohmann_json_j.at("sum0Term"), nlohmann_json_t.sum0Term);
  wpi::adl_serializer<decltype(nlohmann_json_t.sum1Term), void>::from_json(
    nlohmann_json_j.at("sum1Term"), nlohmann_json_t.sum1Term);
  wpi::adl_serializer<decltype(nlohmann_json_t.diff0Term), void>::from_json(
    nlohmann_json_j.at("diff0Term"), nlohmann_json_t.diff0Term);
  wpi::adl_serializer<decltype(nlohmann_json_t.diff1Term), void>::from_json(
    nlohmann_json_j.at("diff1Term"), nlohmann_json_t.diff1Term);
}

}
}
namespace sensors {
inline void
to_json(wpi::json& nlohmann_json_j, const CANCoderFaults& nlohmann_json_t)
{
  nlohmann_json_j["HardwareFault"] = nlohmann_json_t.HardwareFault;
  nlohmann_json_j["APIError"] = nlohmann_json_t.APIError;
  nlohmann_json_j["UnderVoltage"] = nlohmann_json_t.UnderVoltage;
  nlohmann_json_j["ResetDuringEn"] = nlohmann_json_t.ResetDuringEn;
  nlohmann_json_j["MagnetTooWeak"] = nlohmann_json_t.MagnetTooWeak;
}
inline void
from_json(const wpi::json& nlohmann_json_j, CANCoderFaults& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.HardwareFault), void>::from_json(
    nlohmann_json_j.at("HardwareFault"), nlohmann_json_t.HardwareFault);
  wpi::adl_serializer<decltype(nlohmann_json_t.APIError), void>::from_json(
    nlohmann_json_j.at("APIError"), nlohmann_json_t.APIError);
  wpi::adl_serializer<decltype(nlohmann_json_t.UnderVoltage), void>::from_json(
    nlohmann_json_j.at("UnderVoltage"), nlohmann_json_t.UnderVoltage);
  wpi::adl_serializer<decltype(nlohmann_json_t.ResetDuringEn), void>::from_json(
    nlohmann_json_j.at("ResetDuringEn"), nlohmann_json_t.ResetDuringEn);
  wpi::adl_serializer<decltype(nlohmann_json_t.MagnetTooWeak), void>::from_json(
    nlohmann_json_j.at("MagnetTooWeak"), nlohmann_json_t.MagnetTooWeak);
}

inline void
to_json(wpi::json& nlohmann_json_j, const CANCoderStickyFaults& nlohmann_json_t)
{
  nlohmann_json_j["HardwareFault"] = nlohmann_json_t.HardwareFault;
  nlohmann_json_j["APIError"] = nlohmann_json_t.APIError;
  nlohmann_json_j["UnderVoltage"] = nlohmann_json_t.UnderVoltage;
  nlohmann_json_j["ResetDuringEn"] = nlohmann_json_t.ResetDuringEn;
  nlohmann_json_j["MagnetTooWeak"] = nlohmann_json_t.MagnetTooWeak;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          CANCoderStickyFaults& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.HardwareFault), void>::from_json(
    nlohmann_json_j.at("HardwareFault"), nlohmann_json_t.HardwareFault);
  wpi::adl_serializer<decltype(nlohmann_json_t.APIError), void>::from_json(
    nlohmann_json_j.at("APIError"), nlohmann_json_t.APIError);
  wpi::adl_serializer<decltype(nlohmann_json_t.UnderVoltage), void>::from_json(
    nlohmann_json_j.at("UnderVoltage"), nlohmann_json_t.UnderVoltage);
  wpi::adl_serializer<decltype(nlohmann_json_t.ResetDuringEn), void>::from_json(
    nlohmann_json_j.at("ResetDuringEn"), nlohmann_json_t.ResetDuringEn);
  wpi::adl_serializer<decltype(nlohmann_json_t.MagnetTooWeak), void>::from_json(
    nlohmann_json_j.at("MagnetTooWeak"), nlohmann_json_t.MagnetTooWeak);
}

inline void
to_json(wpi::json& nlohmann_json_j,
        const PigeonIMUConfiguration& nlohmann_json_t)
{
  nlohmann_json_j["customParam0"] = nlohmann_json_t.customParam0;
  nlohmann_json_j["customParam1"] = nlohmann_json_t.customParam1;
  nlohmann_json_j["enableOptimizations"] = nlohmann_json_t.enableOptimizations;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          PigeonIMUConfiguration& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.customParam0), void>::from_json(
    nlohmann_json_j.at("customParam0"), nlohmann_json_t.customParam0);
  wpi::adl_serializer<decltype(nlohmann_json_t.customParam1), void>::from_json(
    nlohmann_json_j.at("customParam1"), nlohmann_json_t.customParam1);
  wpi::adl_serializer<decltype(nlohmann_json_t.enableOptimizations),
                      void>::from_json(nlohmann_json_j
                                         .at("enableOptimizations"),
                                       nlohmann_json_t.enableOptimizations);
}

inline void
to_json(wpi::json& nlohmann_json_j,
        const PigeonIMU::FusionStatus& nlohmann_json_t)
{
  nlohmann_json_j["heading"] = nlohmann_json_t.heading;
  nlohmann_json_j["bIsValid"] = nlohmann_json_t.bIsValid;
  nlohmann_json_j["bIsFusing"] = nlohmann_json_t.bIsFusing;
  nlohmann_json_j["description"] = nlohmann_json_t.description;
  nlohmann_json_j["lastError"] = nlohmann_json_t.lastError;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          PigeonIMU::FusionStatus& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.heading), void>::from_json(
    nlohmann_json_j.at("heading"), nlohmann_json_t.heading);
  wpi::adl_serializer<decltype(nlohmann_json_t.bIsValid), void>::from_json(
    nlohmann_json_j.at("bIsValid"), nlohmann_json_t.bIsValid);
  wpi::adl_serializer<decltype(nlohmann_json_t.bIsFusing), void>::from_json(
    nlohmann_json_j.at("bIsFusing"), nlohmann_json_t.bIsFusing);
  wpi::adl_serializer<decltype(nlohmann_json_t.description), void>::from_json(
    nlohmann_json_j.at("description"), nlohmann_json_t.description);
  wpi::adl_serializer<decltype(nlohmann_json_t.lastError), void>::from_json(
    nlohmann_json_j.at("lastError"), nlohmann_json_t.lastError);
}

inline void
to_json(wpi::json& nlohmann_json_j,
        const PigeonIMU::GeneralStatus& nlohmann_json_t)
{
  nlohmann_json_j["state"] = nlohmann_json_t.state;
  nlohmann_json_j["currentMode"] = nlohmann_json_t.currentMode;
  nlohmann_json_j["calibrationError"] = nlohmann_json_t.calibrationError;
  nlohmann_json_j["bCalIsBooting"] = nlohmann_json_t.bCalIsBooting;
  nlohmann_json_j["description"] = nlohmann_json_t.description;
  nlohmann_json_j["tempC"] = nlohmann_json_t.tempC;
  nlohmann_json_j["upTimeSec"] = nlohmann_json_t.upTimeSec;
  nlohmann_json_j["noMotionBiasCount"] = nlohmann_json_t.noMotionBiasCount;
  nlohmann_json_j["tempCompensationCount"] =
    nlohmann_json_t.tempCompensationCount;
  nlohmann_json_j["lastError"] = nlohmann_json_t.lastError;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          PigeonIMU::GeneralStatus& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.state), void>::from_json(
    nlohmann_json_j.at("state"), nlohmann_json_t.state);
  wpi::adl_serializer<decltype(nlohmann_json_t.currentMode), void>::from_json(
    nlohmann_json_j.at("currentMode"), nlohmann_json_t.currentMode);
  wpi::adl_serializer<decltype(nlohmann_json_t.calibrationError),
                      void>::from_json(nlohmann_json_j.at("calibrationError"),
                                       nlohmann_json_t.calibrationError);
  wpi::adl_serializer<decltype(nlohmann_json_t.bCalIsBooting), void>::from_json(
    nlohmann_json_j.at("bCalIsBooting"), nlohmann_json_t.bCalIsBooting);
  wpi::adl_serializer<decltype(nlohmann_json_t.description), void>::from_json(
    nlohmann_json_j.at("description"), nlohmann_json_t.description);
  wpi::adl_serializer<decltype(nlohmann_json_t.tempC), void>::from_json(
    nlohmann_json_j.at("tempC"), nlohmann_json_t.tempC);
  wpi::adl_serializer<decltype(nlohmann_json_t.upTimeSec), void>::from_json(
    nlohmann_json_j.at("upTimeSec"), nlohmann_json_t.upTimeSec);
  wpi::adl_serializer<decltype(nlohmann_json_t.noMotionBiasCount),
                      void>::from_json(nlohmann_json_j.at("noMotionBiasCount"),
                                       nlohmann_json_t.noMotionBiasCount);
  wpi::adl_serializer<decltype(nlohmann_json_t.tempCompensationCount),
                      void>::from_json(nlohmann_json_j
                                         .at("tempCompensationCount"),
                                       nlohmann_json_t.tempCompensationCount);
  wpi::adl_serializer<decltype(nlohmann_json_t.lastError), void>::from_json(
    nlohmann_json_j.at("lastError"), nlohmann_json_t.lastError);
}

inline void
to_json(wpi::json& nlohmann_json_j,
        const CANCoderConfiguration& nlohmann_json_t)
{
  nlohmann_json_j["customParam0"] = nlohmann_json_t.customParam0;
  nlohmann_json_j["customParam1"] = nlohmann_json_t.customParam1;
  nlohmann_json_j["enableOptimizations"] = nlohmann_json_t.enableOptimizations;
  nlohmann_json_j["velocityMeasurementPeriod"] =
    nlohmann_json_t.velocityMeasurementPeriod;
  nlohmann_json_j["velocityMeasurementWindow"] =
    nlohmann_json_t.velocityMeasurementWindow;
  nlohmann_json_j["absoluteSensorRange"] = nlohmann_json_t.absoluteSensorRange;
  nlohmann_json_j["magnetOffsetDegrees"] = nlohmann_json_t.magnetOffsetDegrees;
  nlohmann_json_j["sensorDirection"] = nlohmann_json_t.sensorDirection;
  nlohmann_json_j["initializationStrategy"] =
    nlohmann_json_t.initializationStrategy;
  nlohmann_json_j["sensorCoefficient"] = nlohmann_json_t.sensorCoefficient;
  nlohmann_json_j["unitString"] = nlohmann_json_t.unitString;
  nlohmann_json_j["sensorTimeBase"] = nlohmann_json_t.sensorTimeBase;
}
inline void
from_json(const wpi::json& nlohmann_json_j,
          CANCoderConfiguration& nlohmann_json_t)
{
  wpi::adl_serializer<decltype(nlohmann_json_t.customParam0), void>::from_json(
    nlohmann_json_j.at("customParam0"), nlohmann_json_t.customParam0);
  wpi::adl_serializer<decltype(nlohmann_json_t.customParam1), void>::from_json(
    nlohmann_json_j.at("customParam1"), nlohmann_json_t.customParam1);
  wpi::adl_serializer<decltype(nlohmann_json_t.enableOptimizations),
                      void>::from_json(nlohmann_json_j
                                         .at("enableOptimizations"),
                                       nlohmann_json_t.enableOptimizations);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.velocityMeasurementPeriod),
    void>::from_json(nlohmann_json_j.at("velocityMeasurementPeriod"),
                     nlohmann_json_t.velocityMeasurementPeriod);
  wpi::adl_serializer<
    decltype(nlohmann_json_t.velocityMeasurementWindow),
    void>::from_json(nlohmann_json_j.at("velocityMeasurementWindow"),
                     nlohmann_json_t.velocityMeasurementWindow);
  wpi::adl_serializer<decltype(nlohmann_json_t.absoluteSensorRange),
                      void>::from_json(nlohmann_json_j
                                         .at("absoluteSensorRange"),
                                       nlohmann_json_t.absoluteSensorRange);
  wpi::adl_serializer<decltype(nlohmann_json_t.magnetOffsetDegrees),
                      void>::from_json(nlohmann_json_j
                                         .at("magnetOffsetDegrees"),
                                       nlohmann_json_t.magnetOffsetDegrees);
  wpi::adl_serializer<decltype(nlohmann_json_t.sensorDirection),
                      void>::from_json(nlohmann_json_j.at("sensorDirection"),
                                       nlohmann_json_t.sensorDirection);
  wpi::adl_serializer<decltype(nlohmann_json_t.initializationStrategy),
                      void>::from_json(nlohmann_json_j
                                         .at("initializationStrategy"),
                                       nlohmann_json_t.initializationStrategy);
  wpi::adl_serializer<decltype(nlohmann_json_t.sensorCoefficient),
                      void>::from_json(nlohmann_json_j.at("sensorCoefficient"),
                                       nlohmann_json_t.sensorCoefficient);
  wpi::adl_serializer<decltype(nlohmann_json_t.unitString), void>::from_json(
    nlohmann_json_j.at("unitString"), nlohmann_json_t.unitString);
  wpi::adl_serializer<decltype(nlohmann_json_t.sensorTimeBase),
                      void>::from_json(nlohmann_json_j.at("sensorTimeBase"),
                                       nlohmann_json_t.sensorTimeBase);
}

}
}
}
