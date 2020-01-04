/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "UniversalController.hpp"

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
UniversalController::UniversalController(int port) : GenericHID(port) {
  HAL_Report(HALUsageReporting::kResourceType_Joystick, port);
  
  // Physical Hardware
  xb = new XboxController(port);
  ps = new PS4Controller(port);
}

/**
 * Set the physical controller type
 *
 * @param type ControllerType to use {kXbox, kPS4}
 */
void UniversalController::SetControllerType(ControllerType type){
  m_type = type;
}

/**
 * Get the X axis value of the controller.GenericHID::JoystickHand::kLeftHand
 *
 * @param hand Side of controller whose value should be returned.
 */
double UniversalController::GetX(JoystickHand hand) const {
  if (m_type == ControllerType::kXbox) {
    return xb->GetX(hand);
  } else {
    return ps->GetX(hand);
  }
}

/**
 * Get the Y axis value of the controller.
 *
 * @param hand Side of controller whose value should be returned.
 */
double UniversalController::GetY(JoystickHand hand) const {
  if (m_type == ControllerType::kXbox) {
    return xb->GetY(hand);
  } else {
    return ps->GetY(hand);
  }
}

/**
 * Get the trigger axis value of the controller.
 *
 * @param hand Side of controller whose value should be returned.
 */
double UniversalController::GetTriggerAxis(JoystickHand hand) const {
  if (m_type == ControllerType::kXbox) {
    return xb->GetTriggerAxis(hand);
  } else {
    return ps->GetTriggerAxis(hand);
  }
}

/**
 * Read the value of the bumper button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 */
bool UniversalController::GetBumper(JoystickHand hand) const {
  if (m_type == ControllerType::kXbox) {
    return xb->GetBumper(hand);
  } else {
    return ps->GetBumper(hand);
  }
}

/**
 * Whether the bumper was pressed since the last check.
 *
 * @param hand Side of controller whose value should be returned.
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetBumperPressed(JoystickHand hand) {
  if (m_type == ControllerType::kXbox) {
    return xb->GetBumperPressed(hand);
  } else {
    return ps->GetBumperPressed(hand);
  }
}

/**
 * Whether the bumper was released since the last check.
 *
 * @param hand Side of controller whose value should be returned.
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetBumperReleased(JoystickHand hand) {
  if (m_type == ControllerType::kXbox) {
    return xb->GetBumperReleased(hand);
  } else {
    return ps->GetBumperReleased(hand);
  }
}

/**
 * Read the value of the stick button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetStickButton(JoystickHand hand) const {
  if (m_type == ControllerType::kXbox) {
    return xb->GetStickButton(hand);
  } else {
    return ps->GetStickButton(hand);
  }
}

/**
 * Whether the stick button was pressed since the last check.
 *
 * @param hand Side of controller whose value should be returned.
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetStickButtonPressed(JoystickHand hand) {
  if (m_type == ControllerType::kXbox) {
    return xb->GetStickButtonPressed(hand);
  } else {
    return ps->GetStickButtonPressed(hand);
  }
}

/**
 * Whether the stick button was released since the last check.
 *
 * @param hand Side of controller whose value should be returned.
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetStickButtonReleased(JoystickHand hand) {
  if (m_type == ControllerType::kXbox) {
    return xb->GetStickButtonReleased(hand);
  } else {
    return ps->GetStickButtonReleased(hand);
  }
}

/**
 * Read the value of the Cross button on the controller.
 *
 * @return The state of the button.
 */
bool UniversalController::GetCrossButton() const {
  if (m_type == ControllerType::kXbox) {
    return xb->GetAButton();
  } else {
    return ps->GetCrossButton();
  }
}

/**
 * Whether the Cross button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetCrossButtonPressed() {
  if (m_type == ControllerType::kXbox) {
    return xb->GetAButtonPressed();
  } else {
    return ps->GetCrossButtonPressed();
  }
}

/**
 * Whether the Cross button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetCrossButtonReleased() {
  if (m_type == ControllerType::kXbox) {
    return xb->GetAButtonReleased();
  } else {
    return ps->GetCrossButtonReleased();
  }
}

/**
 * Read the value of the Circle button on the controller.
 *
 * @return The state of the button.
 */
bool UniversalController::GetCircleButton() const {
  if (m_type == ControllerType::kXbox) {
    return xb->GetBButton();
  } else {
    return ps->GetCircleButton();
  }
}

/**
 * Whether the Circle button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetCircleButtonPressed() {
  if (m_type == ControllerType::kXbox) {
    return xb->GetBButtonPressed();
  } else {
    return ps->GetCircleButtonPressed();
  }
}

/**
 * Whether the Circle button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetCircleButtonReleased() {
  if (m_type == ControllerType::kXbox) {
    return xb->GetBButtonReleased();
  } else {
    return ps->GetCircleButtonReleased();
  }
}

/**
 * Read the value of the Square button on the controller.
 *
 * @return The state of the button.
 */
bool UniversalController::GetSquareButton() const {
  if (m_type == ControllerType::kXbox) {
    return xb->GetXButton();
  } else {
    return ps->GetSquareButton();
  };
}

/**
 * Whether the Square button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetSquareButtonPressed() {
  if (m_type == ControllerType::kXbox) {
    return xb->GetXButtonPressed();
  } else {
    return ps->GetSquareButtonPressed();
  }
}

/**
 * Whether the Square button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetSquareButtonReleased() {
  if (m_type == ControllerType::kXbox) {
    return xb->GetXButtonReleased();
  } else {
    return ps->GetSquareButtonReleased();
  }
}

/**
 * Read the value of the Square button on the controller.
 *
 * @return The state of the button.
 */
bool UniversalController::GetTriangleButton() const {
  if (m_type == ControllerType::kXbox) {
    return xb->GetYButton();
  } else {
    return ps->GetTriangleButton();
  }
}

/**
 * Whether the Square button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetTriangleButtonPressed() {
  if (m_type == ControllerType::kXbox) {
    return xb->GetYButtonPressed();
  } else {
    return ps->GetTriangleButtonPressed();
  }
}

/**
 * Whether the Square button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetTriangleButtonReleased() {
  if (m_type == ControllerType::kXbox) {
    return xb->GetYButtonReleased();
  } else {
    return ps->GetTriangleButtonReleased();
  }
}

/**
 * Read the value of the ScreenShot button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetScreenShotButton() const {
  if (m_type == ControllerType::kXbox) {
    return xb->GetBackButton();
  } else {
    return ps->GetScreenShotButton();
  }
}

/**
 * Whether the ScreenShot button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetScreenShotButtonPressed() {
  if (m_type == ControllerType::kXbox) {
    return xb->GetBackButtonPressed();
  } else {
    return ps->GetScreenShotButtonPressed();
  }
}

/**
 * Whether the ScreenShot button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetScreenShotButtonReleased() {
  if (m_type == ControllerType::kXbox) {
    return xb->GetBackButtonReleased();
  } else {
    return ps->GetScreenShotButtonReleased();
  }
}

/**
 * Read the value of the Options button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetOptionsButton() const {
  if (m_type == ControllerType::kXbox) {
    return xb->GetStartButton();
  } else {
    return ps->GetOptionsButton();
  }
}

/**
 * Whether the Options button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetOptionsButtonPressed() {
  if (m_type == ControllerType::kXbox) {
    return xb->GetStartButtonPressed();
  } else {
    return ps->GetOptionsButtonPressed();
  }
}

/**
 * Whether the Options button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetOptionsButtonReleased() {
  if (m_type == ControllerType::kXbox) {
    return xb->GetStartButtonReleased();
  } else {
    return ps->GetOptionsButtonReleased();
  }
}

/**
 * Read the value of the PS button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetPSButton() const {
  if (m_type == ControllerType::kXbox) {
    return false;
  } else {
    return ps->GetPSButton();
  }
}

/**
 * Whether the PS button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetPSButtonPressed() {
  if (m_type == ControllerType::kXbox) {
    return false;
  } else {
    return ps->GetPSButtonPressed();
  }
}

/**
 * Whether the PS button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetPSButtonReleased() {
  if (m_type == ControllerType::kXbox) {
    return false;
  } else {
    return ps->GetPSButtonReleased();
  }
}

/**
 * Read the value of the TouchPad button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetTouchPadButton() const {
  if (m_type == ControllerType::kXbox) {
    return false;
  } else {
    return ps->GetTouchPadButton();
  }
}

/**
 * Whether the TouchPad button was pressed since the last check.
 *
 * @return Whether the button was pressed since the last check.
 */
bool UniversalController::GetTouchPadButtonPressed() {
  if (m_type == ControllerType::kXbox) {
    return false;
  } else {
    return ps->GetTouchPadButtonPressed();
  }
}

/**
 * Whether the PS button was released since the last check.
 *
 * @return Whether the button was released since the last check.
 */
bool UniversalController::GetTouchPadButtonReleased() {
  if (m_type == ControllerType::kXbox) {
    return false;
  } else {
    return ps->GetTouchPadButtonReleased();
  }
}

// TODO: Add XBox Getter functions

/**
 * Read the value of the TouchPad button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetUpButton() const {
  return (GetPOV() == 315 || GetPOV() == 0 || GetPOV() == 45 );
}

/**
 * Read the value of the TouchPad button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetRightButton() const {
  return (GetPOV() == 45 || GetPOV() == 90 || GetPOV() == 135 );
}

/**
 * Read the value of the TouchPad button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetDownButton() const {
  return (GetPOV() == 135 || GetPOV() == 180 || GetPOV() == 225 );
}

/**
 * Read the value of the TouchPad button on the controller.
 *
 * @param hand Side of controller whose value should be returned.
 * @return The state of the button.
 */
bool UniversalController::GetLeftButton() const {
  return (GetPOV() == 225 || GetPOV() == 270 || GetPOV() == 315 );
}