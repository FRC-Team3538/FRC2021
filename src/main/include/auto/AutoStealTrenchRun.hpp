#pragma once

#include <string>

#include <frc/Timer.h>

#include "AutoInterface.hpp"
#include "robotmap.hpp"

class AutoStealTrenchRun : public AutoInterface {
 public:
    // Name of this program, used by SmartDash
    static std::string GetName();

 private:
    // Get a referance to the robotmap
    robotmap& IO;

    // State Variables
    int m_state;   
    Timer m_autoTimer;

    void NextState();

 public:
    // Constructor requires a reference to the RobotMap
    AutoStealTrenchRun() = delete;
    AutoStealTrenchRun(robotmap& );
    ~AutoStealTrenchRun();

    // Auto Program Logic
    void Run();
    void UpdateSmartDash();
};
#pragma once

#include <string>

#include <frc/Timer.h>

#include "AutoInterface.hpp"
#include "robotmap.hpp"

