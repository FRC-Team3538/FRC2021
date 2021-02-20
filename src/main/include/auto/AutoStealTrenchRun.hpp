#pragma once

#include <string>

#include <frc/Timer.h>

#include "AutoInterface.hpp"
#include "RobotMap.hpp"

class AutoStealTrenchRun : public AutoInterface {
 public:
    // Name of this program, used by SmartDash
    static std::string GetName();

 private:
    // Get a referance to the RobotMap
    RobotMap& IO;

    // State Variables
    int m_state;   
    Timer m_autoTimer;

    void NextState();

 public:
    // Constructor requires a reference to the RobotMap
    AutoStealTrenchRun() = delete;
    AutoStealTrenchRun(RobotMap& );
    ~AutoStealTrenchRun();

    // Auto Program Logic
    void Run();
    void UpdateSmartDash();
};
#pragma once

#include <string>

#include <frc/Timer.h>

#include "AutoInterface.hpp"
#include "RobotMap.hpp"

