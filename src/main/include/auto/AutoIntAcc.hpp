#pragma once

#include <string>

#include <frc/Timer.h>

#include "AutoInterface.hpp"
#include "robotmap.hpp"


class AutoIntAcc : public AutoInterface
{
public:
   // Name of this program, used by SmartDash
   static std::string GetName();

private:
   // Get a referance to the robotmap
   robotmap &IO;

   // State Variables
   int m_state;
   Timer m_autoTimer;

   vision::RJVisionPipeline::visionData data;
   int tpCt = 0;
   int shotCounter = 0;
   bool shotCounterOS = false;

   void NextState();

public:
   // Constructor requires a reference to the RobotMap
   AutoIntAcc() = delete;
   AutoIntAcc(robotmap &);
   ~AutoIntAcc();

   // Auto Program Logic
   void Run();
   void UpdateSmartDash();
};