#pragma once

#include <string>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/livewindow/LiveWindow.h>

#include "UniversalController.hpp"

using namespace frc;

class DS
{
  public:
    UniversalController Driver{0};
    UniversalController Operator{1};

    LiveWindow& m_lw = *frc::LiveWindow::GetInstance();

    SendableChooser<std::string> chooseController;
		const std::string sPS4 = "PS4";
		const std::string sXBX = "Xbox";

    SendableChooser<std::string> chooseDriveLimit;
		const std::string sLimit = "Limited";
		const std::string sUnlimitted = "Unlimited";

    DS();
    void SmartDash();

};