#pragma once

#include <string>

#include <frc/Timer.h>

#include "AutoInterface.hpp"
#include "robotmap.hpp"

class AutoTrenchRun : public AutoInterface
{
public:
   // Name of this program, used by SmartDash
   static std::string GetName();

private:
   // Get a referance to the robotmap
   robotmap &IO;

   // State Variables
   int setstate;
   Timer m_autoTimer;

   vision::RJVisionPipeline::visionData data;
   int tpCt = 0;

   void NextState();

public:
   // Constructor requires a reference to the RobotMap
   AutoTrenchRun() = delete;
   AutoTrenchRun(robotmap &);
   ~AutoTrenchRun();

   // Auto Program Logic
   void Run();
   void UpdateSmartDash();
};