#pragma once

#include <string>

#include <frc/Timer.h>

#include "AutoInterface.hpp"
#include "RobotMap.hpp"


class AutoShootTrenchYolo : public AutoInterface
{
public:
   // Name of this program, used by SmartDash
   static std::string GetName();

private:
   // Get a referance to the RobotMap
   RobotMap &IO;

   // State Variables
   int m_state;
   Timer m_autoTimer;

   vision::RJVisionPipeline::visionData data;
   int tpCt = 0;

   void NextState();

public:
   // Constructor requires a reference to the RobotMap
   AutoShootTrenchYolo() = delete;
   AutoShootTrenchYolo(RobotMap &);
   ~AutoShootTrenchYolo();

   // Auto Program Logic
   void Run();
   void UpdateSmartDash();
};