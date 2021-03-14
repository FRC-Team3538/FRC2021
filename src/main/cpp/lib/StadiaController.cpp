/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "lib/StadiaController.hpp"

#include "hal/FRCUsageReporting.h"

using namespace frc;

/**
 * Construct an instance of a Stadia4 controller.
 *
 * The controller index is the USB port on the Driver Station.
 *
 * @param port The port on the Driver Station that the controller is plugged
 *             into (0-5).
 */
StadiaController::StadiaController(int port) : GenericHID(port) {
  HAL_Report(HALUsageReporting::kResourceType_Joystick, port);
}

/**
 * Get the X axis value of the controller.GenericHID::JoystickHand::kLeftHand
 *
 * @param hand Side of controller whose value should be returned.
 */
double StadiaController::GetX(JoystickHand hand) const {
  if (hand == kLeftHand) {
    return GetRawAxis((int) Axis::kLeftX);
  } else {
    return GetRawAxis((int) Axis::kRightX);
  }
}

/**
 * Get the Y axis value of the controller.
 *
 * @param hand Side of controller whose value should be returned.
 */
double StadiaController::GetY(JoystickHand hand) const {
  if (hand == kLeftHand) {
    return GetRawAxis((int) Axis::kLeftY);
  } else {
    return GetRawAxis((int) Axis::kRightY);
  }
}

/**
 * Get the trigger axis value of the controller.
 *
 * @param hand Side of controller whose value should be returned.
 */
double StadiaController::GetTriggerAxis(JoystickHand hand) const {

  // The raw axis reports -1 when not pulled and +1 when fully pulled.
  if (hand == kLeftHand) {
    return GetRawAxis((int) Axis::kLeftTrigger)/2.0 + 0.5;
  } else {
    return GetRawAxis((int) Axis::kRightTrigger)/2.0 + 0.5;
  }
}

/**
 * Read the value of the bumper button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 */
bool StadiaController::GetBumper(JoystickHand hand) const {
  if (hand == kLeftHand) {
    return GetRawButton((int) Button::kLeftBumper);
  } else {
    return GetRawButton((int) Button::kRightBumper);
  }
}

/**
 * Whether the bumper was pressed since the last check.
 *
 * @param hand Side of controller whose value should be returned.
 * @return Whether the button was pressed since the last check.
 */
bool StadiaController::GetBumperPressed(JoystickHand hand) {
  if (hand == kLeftHand) {
    return GetRawButtonPressed((int) Button::kLeftBumper);
  } else {
    return GetRawButtonPressed((int) Button::kRightBumper);
  }
}

/**
 * Whether the bumper was released since the last check.
 *
 * @param hand Side of controller whose value should be returned.
 * @return Whether the button was released since the last check.
 */
bool StadiaController::GetBumperReleased(JoystickHand hand) {
  if (hand == kLeftHand) {
    return GetRawButtonReleased((int) Button::kLeftBumper);
  } else {
    return GetRawButtonReleased((int) Button::kRightBumper);
  }
}

/**
 * Read the value of the stick button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool StadiaController::GetStickButton(JoystickHand hand) const {
  if (hand == kLeftHand) {
    return GetRawButton((int) Button::kL3);
  } else {
    return GetRawButton((int) Button::kR3);
  }
}

/**
 * Whether the stick button was pressed since the last check.
 *
 * @param hand Side of controller whose value should be returned.
 * @return Whether the button was pressed since the last check.
 */
bool StadiaController::GetStickButtonPressed(JoystickHand hand) {
  if (hand == kLeftHand) {
    return GetRawButtonPressed((int) Button::kL3);
  } else {
    return GetRawButtonPressed((int) Button::kR3);
  }
}

/**
 * Whether the stick button was released since the last check.
 *
 * @param hand Side of controller whose value should be returned.
 * @return Whether the button was released since the last check.
 */
bool StadiaController::GetStickButtonReleased(JoystickHand hand) {
  if (hand == kLeftHand) {
    return GetRawButtonReleased((int) Button::kL3);
  } else {
    return GetRawButtonReleased((int) Button::kR3);
  }
}

/**
 * Read the value of the A button on the controller.
 *
 * @return The state of the button.
 */
bool StadiaController::GetAButton() const {
  return GetRawButton((int) Button::kA);
}

/**
 * Whether the A button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool StadiaController::GetAButtonPressed() {
  return GetRawButtonPressed((int) Button::kA);
}

/**
 * Whether the A button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool StadiaController::GetAButtonReleased() {
  return GetRawButtonReleased((int) Button::kA);
}

/**
 * Read the value of the B button on the controller.
 *
 * @return The state of the button.
 */
bool StadiaController::GetBButton() const {
  return GetRawButton((int) Button::kB);
}

/**
 * Whether the B button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool StadiaController::GetBButtonPressed() {
  return GetRawButtonPressed((int) Button::kB);
}

/**
 * Whether the B button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool StadiaController::GetBButtonReleased() {
  return GetRawButtonReleased((int) Button::kB);
}

/**
 * Read the value of the X button on the controller.
 *
 * @return The state of the button.
 */
bool StadiaController::GetXButton() const {
  return GetRawButton((int) Button::kX);
}

/**
 * Whether the X button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool StadiaController::GetXButtonPressed() {
  return GetRawButtonPressed((int) Button::kX);
}

/**
 * Whether the X button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool StadiaController::GetXButtonReleased() {
  return GetRawButtonReleased((int) Button::kX);
}

/**
 * Read the value of the Y button on the controller.
 *
 * @return The state of the button.
 */
bool StadiaController::GetYButton() const {
  return GetRawButton((int) Button::kY);
}

/**
 * Whether the Y button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool StadiaController::GetYButtonPressed() {
  return GetRawButtonPressed((int) Button::kY);
}

/**
 * Whether the Y button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool StadiaController::GetYButtonReleased() {
  return GetRawButtonReleased((int) Button::kY);
}

/**
 * Read the value of the Options button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool StadiaController::GetOptionsButton() const {
  return GetRawButton((int) Button::kOptions);
}

/**
 * Whether the Options button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool StadiaController::GetOptionsButtonPressed() {
  return GetRawButtonPressed((int) Button::kOptions);
}

/**
 * Whether the Options button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool StadiaController::GetOptionsButtonReleased() {
  return GetRawButtonReleased((int) Button::kOptions);
}

/**
 * Read the value of the Menu button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool StadiaController::GetMenuButton() const {
  return GetRawButton((int) Button::kMenu);
}

/**
 * Whether the Menu button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool StadiaController::GetMenuButtonPressed() {
  return GetRawButtonPressed((int) Button::kMenu);
}

/**
 * Whether the Menu button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool StadiaController::GetMenuButtonReleased() {
  return GetRawButtonReleased((int) Button::kMenu);
}

/**
 * Read the value of the Stadia button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool StadiaController::GetStadiaButton() const {
  return GetRawButton((int) Button::kStadia);
}

/**
 * Whether the Stadia button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool StadiaController::GetStadiaButtonPressed() {
  return GetRawButtonPressed((int) Button::kStadia);
}

/**
 * Whether the Stadia button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool StadiaController::GetStadiaButtonReleased() {
  return GetRawButtonReleased((int) Button::kStadia);
}

/**
 * Read the value of the Capture button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool StadiaController::GetCaptureButton() const {
  return GetRawButton((int) Button::kCapture);
}

/**
 * Whether the Capture button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool StadiaController::GetCaptureButtonPressed() {
  return GetRawButtonPressed((int) Button::kCapture);
}

/**
 * Read the value of the Assistant button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool StadiaController::GetAssistantButton() const {
  return GetRawButton((int) Button::kAssistant);
}

/**
 * Whether the Assistant button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool StadiaController::GetAssistantButtonPressed() {
  return GetRawButtonPressed((int) Button::kAssistant);
}

/**
 * Whether the Stadia button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool StadiaController::GetAssistantButtonReleased() {
  return GetRawButtonReleased((int) Button::kAssistant);
}

/**
 * Whether the Stadia button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool StadiaController::GetCaptureButtonReleased() {
  return GetRawButtonReleased((int) Button::kCapture);
}

/**
 * Read the value of the Up button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool StadiaController::GetUpButton() const {
  return (GetPOV() == 315 || GetPOV() == 0 || GetPOV() == 45 );
}

/**
 * Read the value of the Right button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool StadiaController::GetRightButton() const {
  return (GetPOV() == 45 || GetPOV() == 90 || GetPOV() == 135 );
}

/**
 * Read the value of the Down button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool StadiaController::GetDownButton() const {
  return (GetPOV() == 135 || GetPOV() == 180 || GetPOV() == 225 );
}

/**
 * Read the value of the Left button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool StadiaController::GetLeftButton() const {
  return (GetPOV() == 225 || GetPOV() == 270 || GetPOV() == 315 );
}
