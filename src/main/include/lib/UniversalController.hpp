/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include "lib/PS4Controller.hpp"
#include "lib/StadiaController.hpp"

#include <frc/smartdashboard/Sendable.h>
#include <frc/smartdashboard/SendableBuilder.h>
#include <frc/smartdashboard/SendableHelper.h>

namespace frc
{

  /**
 * Handle input from various types of controllers and provide the same 
 * API as a PS4 Controller, since that's what we use. 
 * 
 * TODO: Support other controller APIs as well?
 */
  class UniversalController : public GenericHID,
                              public frc::Sendable,
                              public frc::SendableHelper<UniversalController>
  {
  public:
    explicit UniversalController(int port);
    virtual ~UniversalController() = default;

    UniversalController(const UniversalController &) = delete;
    UniversalController &operator=(const UniversalController &) = delete;

    enum class ControllerType
    {
      kXbox,
      kPS4,
      kStadia,
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

    // SmartDash Support
    void InitSendable(frc::SendableBuilder &builder) override;

  private:
    ControllerType m_type = ControllerType::kPS4;

    XboxController xb_;
    PS4Controller ps_;
    StadiaController stadia_;
  };

} // namespace frc