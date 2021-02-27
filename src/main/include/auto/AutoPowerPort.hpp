#pragma once

#include <string>

#include <frc2/Timer.h>

#include <units/velocity.h>

#include "AutoInterface.hpp"
#include "robotmap.hpp"
#include <lib/DiffyDriveTrajectoryConstraint.hpp>
#include <auto/AutoDrivetrainModel.hpp>

class AutoPowerPort : public AutoInterface
{
public:
   // Name of this program, used by SmartDash
   static std::string GetName();

private:
   // Get a referance to the robotmap
   robotmap &IO;

   // State Variables
   int m_state;
   frc2::Timer m_autoTimer;

   vision::RJVisionPipeline::visionData data;
   int tpCt = 0;
   int shotCounter = 0;
   bool shotCounterOS = false;
   double prevRPM = 0.0;

   void NextState();

   frc::Trajectory m_trajectory;
   frc::Trajectory m_trajectory_rev;
   frc::RamseteController m_ramsete;
   rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper};

public:
   // Constructor requires a reference to the RobotMap
   AutoPowerPort() = delete;
   AutoPowerPort(robotmap &);
   ~AutoPowerPort();

   // Auto Program Logic
   void Init();
   void Run();
   void UpdateSmartDash();
};