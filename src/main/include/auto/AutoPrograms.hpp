#pragma once

#include <string>

#include <frc/smartdashboard/SmartDashboard.h>

#include "robotmap.hpp"
#include "auto/AutoInterface.hpp"
#include <iostream>

class AutoPrograms
{

  private:
    // Get a referance to the robotmap
    robotmap &IO;

    // Selected Auto Program
    AutoInterface* m_autoProgram;

    // SmartDash Chooser
    SendableChooser<std::string> m_chooser;

  public:
    // Constructor requires a reference to the RobotMap
    AutoPrograms() = delete;
    AutoPrograms(robotmap &);

    // Choose a program to Initialize
    void Init();

    // Run the selected program
    void Run();
    void SmartDash();
};