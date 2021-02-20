/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "PS4Controller.hpp"

#include "hal/FRCUsageReporting.h"

using namespace frc;

/**
 * Construct an instance of a PS4 controller.
 *
 * The controller index is the USB port on the Driver Station.
 *
 * @param port The port on the Driver Station that the controller is plugged
 *             into (0-5).
 */
PS4Controller::PS4Controller(int port) : GenericHID(port) {
  HAL_Report(HALUsageReporting::kResourceType_Joystick, port);
}

/**
 * Get the X axis value of the controller.GenericHID::JoystickHand::kLeftHand
 *
 * @param hand Side of controller whose value should be returned.
 */
double PS4Controller::GetX(JoystickHand hand) const {
  if (hand == kLeftHand) {
    return GetRawAxis(0);
  } else {
    return GetRawAxis(2);
  }
}

/**
 * Get the Y axis value of the controller.
 *
 * @param hand Side of controller whose value should be returned.
 */
double PS4Controller::GetY(JoystickHand hand) const {
  if (hand == kLeftHand) {
    return GetRawAxis(1);
  } else {
    return GetRawAxis(5);
  }
}

/**
 * Get the trigger axis value of the controller.
 *
 * @param hand Side of controller whose value should be returned.
 */
double PS4Controller::GetTriggerAxis(JoystickHand hand) const {
  if (GetRawAxis(3) == 0.0 && GetRawAxis(4) == 0.0){
    //Controller is unplugged
    return 0.0;
  } 

  if (hand == kLeftHand) {
    return GetRawAxis(3)/2.0 + 0.5;
  } else {
    return GetRawAxis(4)/2.0 + 0.5;
  }
}

/**
 * Read the value of the bumper button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 */
bool PS4Controller::GetBumper(JoystickHand hand) const {
  if (hand == kLeftHand) {
    return GetRawButton(static_cast<int>(Button::kBumperLeft));
  } else {
    return GetRawButton(static_cast<int>(Button::kBumperRight));
  }
}

/**
 * Whether the bumper was pressed since the last check.
 *
 * @param hand Side of controller whose value should be returned.
 * @return Whether the button was pressed since the last check.
 */
bool PS4Controller::GetBumperPressed(JoystickHand hand) {
  if (hand == kLeftHand) {
    return GetRawButtonPressed(static_cast<int>(Button::kBumperLeft));
  } else {
    return GetRawButtonPressed(static_cast<int>(Button::kBumperRight));
  }
}

/**
 * Whether the bumper was released since the last check.
 *
 * @param hand Side of controller whose value should be returned.
 * @return Whether the button was released since the last check.
 */
bool PS4Controller::GetBumperReleased(JoystickHand hand) {
  if (hand == kLeftHand) {
    return GetRawButtonReleased(static_cast<int>(Button::kBumperLeft));
  } else {
    return GetRawButtonReleased(static_cast<int>(Button::kBumperRight));
  }
}

/**
 * Read the value of the stick button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool PS4Controller::GetStickButton(JoystickHand hand) const {
  if (hand == kLeftHand) {
    return GetRawButton(static_cast<int>(Button::kStickLeft));
  } else {
    return GetRawButton(static_cast<int>(Button::kStickRight));
  }
}

/**
 * Whether the stick button was pressed since the last check.
 *
 * @param hand Side of controller whose value should be returned.
 * @return Whether the button was pressed since the last check.
 */
bool PS4Controller::GetStickButtonPressed(JoystickHand hand) {
  if (hand == kLeftHand) {
    return GetRawButtonPressed(static_cast<int>(Button::kStickLeft));
  } else {
    return GetRawButtonPressed(static_cast<int>(Button::kStickRight));
  }
}

/**
 * Whether the stick button was released since the last check.
 *
 * @param hand Side of controller whose value should be returned.
 * @return Whether the button was released since the last check.
 */
bool PS4Controller::GetStickButtonReleased(JoystickHand hand) {
  if (hand == kLeftHand) {
    return GetRawButtonReleased(static_cast<int>(Button::kStickLeft));
  } else {
    return GetRawButtonReleased(static_cast<int>(Button::kStickRight));
  }
}

/**
 * Read the value of the Cross button on the controller.
 *
 * @return The state of the button.
 */
bool PS4Controller::GetCrossButton() const {
  return GetRawButton(static_cast<int>(Button::kCross));
}

/**
 * Whether the Cross button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool PS4Controller::GetCrossButtonPressed() {
  return GetRawButtonPressed(static_cast<int>(Button::kCross));
}

/**
 * Whether the Cross button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool PS4Controller::GetCrossButtonReleased() {
  return GetRawButtonReleased(static_cast<int>(Button::kCross));
}

/**
 * Read the value of the Circle button on the controller.
 *
 * @return The state of the button.
 */
bool PS4Controller::GetCircleButton() const {
  return GetRawButton(static_cast<int>(Button::kCircle));
}

/**
 * Whether the Circle button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool PS4Controller::GetCircleButtonPressed() {
  return GetRawButtonPressed(static_cast<int>(Button::kCircle));
}

/**
 * Whether the Circle button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool PS4Controller::GetCircleButtonReleased() {
  return GetRawButtonReleased(static_cast<int>(Button::kCircle));
}

/**
 * Read the value of the Square button on the controller.
 *
 * @return The state of the button.
 */
bool PS4Controller::GetSquareButton() const {
  return GetRawButton(static_cast<int>(Button::kSquare));
}

/**
 * Whether the Square button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool PS4Controller::GetSquareButtonPressed() {
  return GetRawButtonPressed(static_cast<int>(Button::kSquare));
}

/**
 * Whether the Square button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool PS4Controller::GetSquareButtonReleased() {
  return GetRawButtonReleased(static_cast<int>(Button::kSquare));
}

/**
 * Read the value of the Square button on the controller.
 *
 * @return The state of the button.
 */
bool PS4Controller::GetTriangleButton() const {
  return GetRawButton(static_cast<int>(Button::kTriangle));
}

/**
 * Whether the Square button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool PS4Controller::GetTriangleButtonPressed() {
  return GetRawButtonPressed(static_cast<int>(Button::kTriangle));
}

/**
 * Whether the Square button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool PS4Controller::GetTriangleButtonReleased() {
  return GetRawButtonReleased(static_cast<int>(Button::kTriangle));
}

/**
 * Read the value of the ScreenShot button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool PS4Controller::GetScreenShotButton() const {
  return GetRawButton(static_cast<int>(Button::kScreenShot));
}

/**
 * Whether the ScreenShot button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool PS4Controller::GetScreenShotButtonPressed() {
  return GetRawButtonPressed(static_cast<int>(Button::kScreenShot));
}

/**
 * Whether the ScreenShot button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool PS4Controller::GetScreenShotButtonReleased() {
  return GetRawButtonReleased(static_cast<int>(Button::kScreenShot));
}

/**
 * Read the value of the Options button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool PS4Controller::GetOptionsButton() const {
  return GetRawButton(static_cast<int>(Button::kOptions));
}

/**
 * Whether the Options button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool PS4Controller::GetOptionsButtonPressed() {
  return GetRawButtonPressed(static_cast<int>(Button::kOptions));
}

/**
 * Whether the Options button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool PS4Controller::GetOptionsButtonReleased() {
  return GetRawButtonReleased(static_cast<int>(Button::kOptions));
}

/**
 * Read the value of the PS button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool PS4Controller::GetPSButton() const {
  return GetRawButton(static_cast<int>(Button::kPS));
}

/**
 * Whether the PS button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool PS4Controller::GetPSButtonPressed() {
  return GetRawButtonPressed(static_cast<int>(Button::kPS));
}

/**
 * Whether the PS button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool PS4Controller::GetPSButtonReleased() {
  return GetRawButtonReleased(static_cast<int>(Button::kPS));
}

/**
 * Read the value of the TouchPad button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool PS4Controller::GetTouchPadButton() const {
  return GetRawButton(static_cast<int>(Button::kTouchPad));
}

/**
 * Whether the TouchPad button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool PS4Controller::GetTouchPadButtonPressed() {
  return GetRawButtonPressed(static_cast<int>(Button::kTouchPad));
}

/**
 * Whether the PS button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool PS4Controller::GetTouchPadButtonReleased() {
  return GetRawButtonReleased(static_cast<int>(Button::kTouchPad));
}

/**
 * Read the value of the TouchPad button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool PS4Controller::GetUpButton() const {
  return (GetPOV() == 315 || GetPOV() == 0 || GetPOV() == 45 );
}

/**
 * Read the value of the TouchPad button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool PS4Controller::GetRightButton() const {
  return (GetPOV() == 45 || GetPOV() == 90 || GetPOV() == 135 );
}

/**
 * Read the value of the TouchPad button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool PS4Controller::GetDownButton() const {
  return (GetPOV() == 135 || GetPOV() == 180 || GetPOV() == 225 );
}

/**
 * Read the value of the TouchPad button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool PS4Controller::GetLeftButton() const {
  return (GetPOV() == 225 || GetPOV() == 270 || GetPOV() == 315 );
}
