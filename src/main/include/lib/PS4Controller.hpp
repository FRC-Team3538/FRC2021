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
class PS4Controller : public GenericHID {
 public:
  explicit PS4Controller(int port);
  virtual ~PS4Controller() = default;

  PS4Controller(const PS4Controller&) = delete;
  PS4Controller& operator=(const PS4Controller&) = delete;

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

  bool GetShareButton() const;
  bool GetShareButtonPressed();
  bool GetShareButtonReleased();

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
  enum class Button {
    kSquare = 1,
    kCross = 2,
    kCircle = 3,
    kTriangle = 4,
    kBumperLeft = 5,
    kBumperRight = 6,
    kShare = 9,
    kOptions = 10,
    kStickLeft = 11,
    kStickRight = 12,
    kPS = 13,
    kTouchPad = 14
  };

  enum class POV {
    kUP = 0,
    kRight = 90,
    kDown = 180,
    kLeft = 270
  };

};

}  // namespace frc