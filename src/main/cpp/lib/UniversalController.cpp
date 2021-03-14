/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/UniversalController.hpp"

#include "hal/FRCUsageReporting.h"

using namespace frc;

/**
 * Construct an instance of a ps_4 controller.
 *
 * The controller index is the USB port on the Driver Station.
 *
 * @param port The port on the Driver Station that the controller is plugged
 *             into (0-5).
 */
UniversalController::UniversalController(int port) : GenericHID(port),
                                                     xb_{port},
                                                     ps_{port},
                                                     stadia_{port}
{
  HAL_Report(HALUsageReporting::kResourceType_Joystick, port);
}

/**
 * Set the physical controller type
 *
 * @param type ControllerType to use {kXbox, kPS4, kStadia}
 */
void UniversalController::SetControllerType(ControllerType type)
{
  m_type = type;
}

/**
 * Get the X axis value of the controller.GenericHID::JoystickHand::kLeftHand
 *
 * @param hand Side of controller whose value should be returned.
 */
double UniversalController::GetX(JoystickHand hand) const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetX(hand);
  case ControllerType::kPS4:
    return ps_.GetX(hand);
  case ControllerType::kStadia:
    return stadia_.GetX(hand);
  default:
    return 0.0;
  }
}

/**
 * Get the Y axis value of the controller.
 *
 * @param hand Side of controller whose value should be returned.
 */
double UniversalController::GetY(JoystickHand hand) const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetY(hand);
  case ControllerType::kPS4:
    return ps_.GetY(hand);
  case ControllerType::kStadia:
    return stadia_.GetY(hand);
  default:
    return 0.0;
  }
}

/**
 * Get the trigger axis value of the controller.
 *
 * @param hand Side of controller whose value should be returned.
 */
double UniversalController::GetTriggerAxis(JoystickHand hand) const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetTriggerAxis(hand);
  case ControllerType::kPS4:
    return ps_.GetTriggerAxis(hand);
  case ControllerType::kStadia:
    return stadia_.GetTriggerAxis(hand);
  default:
    return 0.0;
  }
}

/**
 * Read the value of the bumper button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 */
bool UniversalController::GetBumper(JoystickHand hand) const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetBumper(hand);
  case ControllerType::kPS4:
    return ps_.GetBumper(hand);
  case ControllerType::kStadia:
    return stadia_.GetBumper(hand);
  default:
    return false;
  }
}

/**
 * Whether the bumper was pressed since the last check.
 *
 * @param hand Side of controller whose value should be returned.
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetBumperPressed(JoystickHand hand)
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetBumperPressed(hand);
  case ControllerType::kPS4:
    return ps_.GetBumperPressed(hand);
  case ControllerType::kStadia:
    return stadia_.GetBumperPressed(hand);
  default:
    return false;
  }
}

/**
 * Whether the bumper was released since the last check.
 *
 * @param hand Side of controller whose value should be returned.
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetBumperReleased(JoystickHand hand)
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetBumperReleased(hand);
  case ControllerType::kPS4:
    return ps_.GetBumperReleased(hand);
  case ControllerType::kStadia:
    return stadia_.GetBumperReleased(hand);
  default:
    return false;
  }
}

/**
 * Read the value of the stick button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetStickButton(JoystickHand hand) const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetStickButton(hand);
  case ControllerType::kPS4:
    return ps_.GetStickButton(hand);
  case ControllerType::kStadia:
    return stadia_.GetStickButton(hand);
  default:
    return false;
  }
}

/**
 * Whether the stick button was pressed since the last check.
 *
 * @param hand Side of controller whose value should be returned.
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetStickButtonPressed(JoystickHand hand)
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetStickButtonPressed(hand);
  case ControllerType::kPS4:
    return ps_.GetStickButtonPressed(hand);
  case ControllerType::kStadia:
    return stadia_.GetStickButtonPressed(hand);
  default:
    return false;
  }
}

/**
 * Whether the stick button was released since the last check.
 *
 * @param hand Side of controller whose value should be returned.
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetStickButtonReleased(JoystickHand hand)
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetStickButtonReleased(hand);
  case ControllerType::kPS4:
    return ps_.GetStickButtonReleased(hand);
  case ControllerType::kStadia:
    return stadia_.GetStickButtonReleased(hand);
  default:
    return false;
  }
}

/**
 * Read the value of the Cross button on the controller.
 *
 * @return The state of the button.
 */
bool UniversalController::GetCrossButton() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetAButton();
  case ControllerType::kPS4:
    return ps_.GetCrossButton();
  case ControllerType::kStadia:
    return stadia_.GetAButton();
  default:
    return false;
  }
}

/**
 * Whether the Cross button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetCrossButtonPressed()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetAButtonPressed();
  case ControllerType::kPS4:
    return ps_.GetCrossButtonPressed();
  case ControllerType::kStadia:
    return stadia_.GetAButtonPressed();
  default:
    return false;
  }
}

/**
 * Whether the Cross button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetCrossButtonReleased()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetAButtonReleased();
  case ControllerType::kPS4:
    return ps_.GetCrossButtonReleased();
  case ControllerType::kStadia:
    return stadia_.GetAButtonReleased();
  default:
    return false;
  }
}

/**
 * Read the value of the Circle button on the controller.
 *
 * @return The state of the button.
 */
bool UniversalController::GetCircleButton() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetBButton();
  case ControllerType::kPS4:
    return ps_.GetCircleButton();
  case ControllerType::kStadia:
    return stadia_.GetBButton();
  default:
    return false;
  }
}

/**
 * Whether the Circle button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetCircleButtonPressed()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetBButtonPressed();
  case ControllerType::kPS4:
    return ps_.GetCircleButtonPressed();
  case ControllerType::kStadia:
    return stadia_.GetBButtonPressed();
  default:
    return false;
  }
}

/**
 * Whether the Circle button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetCircleButtonReleased()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetBButtonReleased();
  case ControllerType::kPS4:
    return ps_.GetCircleButtonReleased();
  case ControllerType::kStadia:
    return stadia_.GetBButtonReleased();
  default:
    return false;
  }
}

/**
 * Read the value of the Square button on the controller.
 *
 * @return The state of the button.
 */
bool UniversalController::GetSquareButton() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetXButton();
  case ControllerType::kPS4:
    return ps_.GetSquareButton();
  case ControllerType::kStadia:
    return stadia_.GetXButton();
  default:
    return false;
  }
}

/**
 * Whether the Square button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetSquareButtonPressed()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetXButtonPressed();
  case ControllerType::kPS4:
    return ps_.GetSquareButtonPressed();
  case ControllerType::kStadia:
    return stadia_.GetXButtonPressed();
  default:
    return false;
  }
}

/**
 * Whether the Square button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetSquareButtonReleased()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetXButtonReleased();
  case ControllerType::kPS4:
    return ps_.GetSquareButtonReleased();
  case ControllerType::kStadia:
    return stadia_.GetXButtonReleased();
  default:
    return false;
  }
}

/**
 * Read the value of the Square button on the controller.
 *
 * @return The state of the button.
 */
bool UniversalController::GetTriangleButton() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetYButton();
  case ControllerType::kPS4:
    return ps_.GetTriangleButton();
  case ControllerType::kStadia:
    return stadia_.GetYButton();
  default:
    return false;
  }
}

/**
 * Whether the Square button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetTriangleButtonPressed()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetYButtonPressed();
  case ControllerType::kPS4:
    return ps_.GetTriangleButtonPressed();
  case ControllerType::kStadia:
    return stadia_.GetYButtonPressed();
  default:
    return false;
  }
}

/**
 * Whether the Square button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetTriangleButtonReleased()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetYButtonReleased();
  case ControllerType::kPS4:
    return ps_.GetTriangleButtonReleased();
  case ControllerType::kStadia:
    return stadia_.GetYButtonReleased();
  default:
    return false;
  }
}

/**
 * Read the value of the ScreenShot button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetScreenShotButton() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetBackButton();
  case ControllerType::kPS4:
    return ps_.GetScreenShotButton();
  case ControllerType::kStadia:
    return stadia_.GetOptionsButton();
  default:
    return false;
  }
}

/**
 * Whether the ScreenShot button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetScreenShotButtonPressed()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetBackButtonPressed();
  case ControllerType::kPS4:
    return ps_.GetScreenShotButtonPressed();
  case ControllerType::kStadia:
    return stadia_.GetOptionsButtonPressed();
  default:
    return false;
  }
}

/**
 * Whether the ScreenShot button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetScreenShotButtonReleased()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetBackButtonReleased();
  case ControllerType::kPS4:
    return ps_.GetScreenShotButtonReleased();
  case ControllerType::kStadia:
    return stadia_.GetOptionsButtonReleased();
  default:
    return false;
  }
}

/**
 * Read the value of the Options button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetOptionsButton() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetStartButton();
  case ControllerType::kPS4:
    return ps_.GetOptionsButton();
  case ControllerType::kStadia:
    return stadia_.GetMenuButton();
  default:
    return false;
  }
}

/**
 * Whether the Options button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetOptionsButtonPressed()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetStartButtonPressed();
  case ControllerType::kPS4:
    return ps_.GetOptionsButtonPressed();
  case ControllerType::kStadia:
    return stadia_.GetMenuButtonPressed();
  default:
    return false;
  }
}

/**
 * Whether the Options button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetOptionsButtonReleased()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return xb_.GetStartButtonReleased();
  case ControllerType::kPS4:
    return ps_.GetOptionsButtonReleased();
  case ControllerType::kStadia:
    return stadia_.GetMenuButtonReleased();
  default:
    return false;
  }
}

/**
 * Read the value of the ps_ button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetPSButton() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return false;
  case ControllerType::kPS4:
    return ps_.GetPSButton();
  case ControllerType::kStadia:
    return stadia_.GetStadiaButton();
  default:
    return false;
  }
}

/**
 * Whether the ps_ button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetPSButtonPressed()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return false;
  case ControllerType::kPS4:
    return ps_.GetPSButtonPressed();
  case ControllerType::kStadia:
    return stadia_.GetStadiaButtonPressed();
  default:
    return false;
  }
}

/**
 * Whether the ps_ button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetPSButtonReleased()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return false;
  case ControllerType::kPS4:
    return ps_.GetPSButtonReleased();
  case ControllerType::kStadia:
    return stadia_.GetStadiaButtonReleased();
  default:
    return false;
  }
}

/**
 * Read the value of the TouchPad button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetTouchPadButton() const
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return false;
  case ControllerType::kPS4:
    return ps_.GetTouchPadButton();
  case ControllerType::kStadia:
    return false;
  default:
    return false;
  }
}

/**
 * Whether the TouchPad button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetTouchPadButtonPressed()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return false;
  case ControllerType::kPS4:
    return ps_.GetTouchPadButtonPressed();
  case ControllerType::kStadia:
    return false;
  default:
    return false;
  }
}

/**
 * Whether the ps_ button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetTouchPadButtonReleased()
{
  switch (m_type)
  {
  case ControllerType::kXbox:
    return false;
  case ControllerType::kPS4:
    return ps_.GetTouchPadButtonReleased();
  case ControllerType::kStadia:
    return false;
  default:
    return false;
  }
}

// TODO: Add xb_ox Getter functions

/**
 * Read the value of the TouchPad button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetUpButton() const
{
  return (GetPOV() == 315 || GetPOV() == 0 || GetPOV() == 45);
}

/**
 * Read the value of the TouchPad button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetRightButton() const
{
  return (GetPOV() == 45 || GetPOV() == 90 || GetPOV() == 135);
}

/**
 * Read the value of the TouchPad button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetDownButton() const
{
  return (GetPOV() == 135 || GetPOV() == 180 || GetPOV() == 225);
}

/**
 * Read the value of the TouchPad button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetLeftButton() const
{
  return (GetPOV() == 225 || GetPOV() == 270 || GetPOV() == 315);
}

/**
 * Generate a SmartDash object interface
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
void UniversalController::InitSendable(frc::SendableBuilder &builder)
{
  builder.SetSmartDashboardType("UniversalController");
  builder.SetActuator(true);

  // Axis
  builder.AddDoubleProperty(
      "axis/LX", [this] { return GetX(JoystickHand::kLeftHand); }, nullptr);
  builder.AddDoubleProperty(
      "axis/LY", [this] { return GetY(JoystickHand::kLeftHand); }, nullptr);
  builder.AddDoubleProperty(
      "axis/LT", [this] { return GetTriggerAxis(JoystickHand::kLeftHand); }, nullptr);
  builder.AddDoubleProperty(
      "axis/RX", [this] { return GetX(JoystickHand::kRightHand); }, nullptr);
  builder.AddDoubleProperty(
      "axis/RY", [this] { return GetY(JoystickHand::kRightHand); }, nullptr);
  builder.AddDoubleProperty(
      "axis/RT", [this] { return GetTriggerAxis(JoystickHand::kRightHand); }, nullptr);
  builder.AddDoubleProperty(
      "axis/POV", [this] { return GetPOV(); }, nullptr);

  // Button
  builder.AddBooleanProperty(
      "btn/Cross", [this] { return GetCrossButton(); }, nullptr);
  builder.AddBooleanProperty(
      "btn/Circle", [this] { return GetCircleButton(); }, nullptr);
  builder.AddBooleanProperty(
      "btn/Square", [this] { return GetSquareButton(); }, nullptr);
  builder.AddBooleanProperty(
      "btn/Triangle", [this] { return GetTriangleButton(); }, nullptr);

  builder.AddBooleanProperty(
      "btn/ScreenShot", [this] { return GetScreenShotButton(); }, nullptr);
  builder.AddBooleanProperty(
      "btn/Options", [this] { return GetOptionsButton(); }, nullptr);
  builder.AddBooleanProperty(
      "btn/PS", [this] { return GetPSButton(); }, nullptr);
  builder.AddBooleanProperty(
      "btn/Touchpad", [this] { return GetTouchPadButton(); }, nullptr);

}