#pragma once

#include <string>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/XboxController.h>
#include "PS4Controller.hpp"
#include <string>
using namespace frc;

class DS
{
  public:
    XboxController DriverXB{0};
    XboxController OperatorXB{1};

    PS4Controller DriverPS{0};
    PS4Controller OperatorPS{1};

    LiveWindow& m_lw = *frc::LiveWindow::GetInstance();

    SendableChooser<std::string> chooseController;
		const std::string sPS4 = "PS4";
		const std::string sXBX = "Xbox";

    SendableChooser<std::string> chooseDriveLimit;
		const std::string sLimit = "Limited";
		const std::string sUnlimitted = "Unlimited";


    DS();
    void SmartDash();
    private:

};