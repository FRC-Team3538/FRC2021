/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/GenericHID.h>

namespace frc {

/**
 * Handle input from PS4 controllers connected to the Driver Station.
 *
 * This class handles input that comes from the Driver Station. Each time a
 * value is requested the most recent value is returned. There is a single class
 * instance for each controller and the mapping of ports to hardware buttons
 * depends on the code in the Driver Station.
 */
class StadiaController : public GenericHID {
 public:
  explicit StadiaController(int port);
  virtual ~StadiaController() = default;

  StadiaController(const StadiaController&) = delete;
  StadiaController& operator=(const StadiaController&) = delete;

  double GetX(JoystickHand hand) const override;
  double GetY(JoystickHand hand) const override;
  double GetTriggerAxis(JoystickHand hand) const;

  bool GetBumper(JoystickHand hand) const;
  bool GetBumperPressed(JoystickHand hand);
  bool GetBumperReleased(JoystickHand hand);

  bool GetStickButton(JoystickHand hand) const;
  bool GetStickButtonPressed(JoystickHand hand);
  bool GetStickButtonReleased(JoystickHand hand);

  bool GetAButton() const;
  bool GetAButtonPressed();
  bool GetAButtonReleased();

  bool GetBButton() const;
  bool GetBButtonPressed();
  bool GetBButtonReleased();

  bool GetXButton() const;
  bool GetXButtonPressed();
  bool GetXButtonReleased();

  bool GetYButton() const;
  bool GetYButtonPressed();
  bool GetYButtonReleased();

  bool GetOptionsButton() const;
  bool GetOptionsButtonPressed();
  bool GetOptionsButtonReleased();

  bool GetMenuButton() const;
  bool GetMenuButtonPressed();
  bool GetMenuButtonReleased();

  bool GetAssistantButton() const;
  bool GetAssistantButtonPressed();
  bool GetAssistantButtonReleased();

  bool GetCaptureButton() const;
  bool GetCaptureButtonPressed();
  bool GetCaptureButtonReleased();

  bool GetStadiaButton() const;
  bool GetStadiaButtonPressed();
  bool GetStadiaButtonReleased();

  bool GetUpButton() const;
  bool GetRightButton() const;
  bool GetDownButton() const;
  bool GetLeftButton() const;

 private:
  enum class Axis {
    kLeftX = 0,
    kLeftY = 1,
    kRightX = 2,
    kRightY = 3,
    kRightTrigger = 4,
    kLeftTrigger = 5,
  };

  enum class Button {
    kA = 0,
    kB = 1,
    kX = 2,
    kY = 3,
    kLeftBumper = 4,
    kRightBumper = 5,
    kOptions = 6,
    kMenu = 7,
    kStadia = 8,
    kAssistant = 9,
    kCapture = 10,
    kL3= 11,
    kR3 = 12,
    kRightTrigger = 13,
    kLeftTrigger = 14,
  };

  enum class POV {
    kUP = 0,
    kRight = 90,
    kDown = 180,
    kLeft = 270,
  };

};

}  // namespace frc