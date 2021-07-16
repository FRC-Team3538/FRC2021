#pragma once

#include <string>

#include <frc2/Timer.h>

#include <units/velocity.h>

#include "AutoInterface.hpp"
#include "robotmap.hpp"
#include <lib/DiffyDriveTrajectoryConstraint.hpp>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include "DrivetrainModel.hpp"
#include <wpi/json.h>
#include <memory>

class YoinkyMcSteal : public AutoInterface
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

    void NextState();

    frc::Trajectory m_trajectory;
    frc::Trajectory m_trajectorydos;
    frc::Trajectory m_trajectorytres;
    frc::Trajectory m_trajectorycuatro;

    vision::RJVisionPipeline::visionData data;
    int tpCt = 0;
    int shotCounter = 0;
    bool shotCounterOS = false;

public:
    // Constructor requires a reference to the RobotMap
    YoinkyMcSteal() = delete;
    YoinkyMcSteal(robotmap &);
    ~YoinkyMcSteal();

    // Auto Program Logic
    void Init();
    void Run();
    void UpdateSmartDash();
};