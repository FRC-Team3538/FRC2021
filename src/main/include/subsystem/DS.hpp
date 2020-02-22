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
		const std::string sPS4 = "Both PS4 Controllers";
		const std::string sXBX = "Both Xbox Controllers";
    const std::string sPS4Driver = "PS4 Driver Xbox Operator";
    const std::string sPS4Operator = "PS4 Operator";
    const std::string sXboxDriver = "Xbox Driver PS4 Operator";
    const std::string sXboxOperator = "Xbox Operator";


    DS();
    void SmartDash();

};