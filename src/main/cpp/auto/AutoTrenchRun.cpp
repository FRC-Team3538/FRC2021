#include "auto/AutoTrenchRun.hpp"

// Name for Smart Dash Chooser
std::string AutoTrenchRun::GetName()
{
    return "2 - AutoTrenchRun";
}

// Initialization
// Constructor requires a reference to the robot map
AutoTrenchRun::AutoTrenchRun(robotmap &IO) : IO(IO)
{
    m_state = 0;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    IO.drivebase.Stop();
}

AutoTrenchRun::~AutoTrenchRun() { }


//State Machine
void AutoTrenchRun::NextState(){
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

// Execute the program
void AutoTrenchRun::Run()
{
    switch (m_state)
    {
    case 0:
    {
        IO.drivebase.Stop();
        IO.shooter.SetShooterDistance(144.0);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 1:
    {
        IO.drivebase.TurnAbs(15);
        IO.shooter.IntakeDeploy();
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 2:
    {
        IO.drivebase.DriveForward(-180);
        IO.shooter.SetIntake(0.5);
        IO.shooter.SetIndexer(0.5);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 3: {
        IO.drivebase.DriveForward(108);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 4: {
        IO.drivebase.TurnAbs(0);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
     case 5: {
        IO.shooter.SetShooterDistance(204);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    
    
    
    default:
        IO.drivebase.Stop();
        IO.shooter.Stop();
    }
}