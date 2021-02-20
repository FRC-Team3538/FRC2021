/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include "PS4Controller.hpp"

namespace frc {

/**
 * Handle input from PS4 controllers connected to the Driver Station.
 *
 * This class handles input that comes from the Driver Station. Each time a
 * value is requested the most recent value is returned. There is a single class
 * instance for each controller and the mapping of ports to hardware buttons
 * depends on the code in the Driver Station.
 */
class UniversalController : public GenericHID {
 public:
  explicit UniversalController(int port);
  virtual ~UniversalController() = default;

  UniversalController(const UniversalController&) = delete;
  UniversalController& operator=(const UniversalController&) = delete;

  enum class ControllerType {
    kXbox = 0,
    kPS4
  };
  void SetControllerType(ControllerType type);

  double GetX(JoystickHand hand) const override;
  double GetY(JoystickHand hand) const override;
  double GetTriggerAxis(JoystickHand hand) const;

  bool GetBumper(JoystickHand hand) const;
  bool GetBumperPressed(JoystickHand hand);
  bool GetBumperReleased(JoystickHand hand);

  bool GetStickButton(JoystickHand hand) const;
  bool GetStickButtonPressed(JoystickHand hand);
  bool GetStickButtonReleased(JoystickHand hand);

  bool GetCrossButton() const;
  bool GetCrossButtonPressed();
  bool GetCrossButtonReleased();

  bool GetCircleButton() const;
  bool GetCircleButtonPressed();
  bool GetCircleButtonReleased();

  bool GetSquareButton() const;
  bool GetSquareButtonPressed();
  bool GetSquareButtonReleased();

  bool GetTriangleButton() const;
  bool GetTriangleButtonPressed();
  bool GetTriangleButtonReleased();

  bool GetScreenShotButton() const;
  bool GetScreenShotButtonPressed();
  bool GetScreenShotButtonReleased();

  bool GetOptionsButton() const;
  bool GetOptionsButtonPressed();
  bool GetOptionsButtonReleased();

  bool GetPSButton() const;
  bool GetPSButtonPressed();
  bool GetPSButtonReleased();

  bool GetTouchPadButton() const;
  bool GetTouchPadButtonPressed();
  bool GetTouchPadButtonReleased();

  bool GetUpButton() const;
  bool GetRightButton() const;
  bool GetDownButton() const;
  bool GetLeftButton() const;

 private:
  ControllerType m_type = ControllerType::kPS4;

  XboxController* xb;
  PS4Controller* ps;

};

}  // namespace frc