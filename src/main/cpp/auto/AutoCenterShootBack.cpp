#include "auto/AutoCenterShootBack.hpp"

// Name for Smart Dash Chooser
std::string AutoCenterShootBack::GetName()
{
    return "3 - AutoCenterShootBack";
}

// Initialization
// Constructor requires a reference to the robot map
AutoCenterShootBack::AutoCenterShootBack(robotmap &IO) : IO(IO)
{
    m_state = 0;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    IO.drivebase.Stop();
}

AutoCenterShootBack::~AutoCenterShootBack() { }

//State Machine
void AutoCenterShootBack::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

// Execute the program
void AutoCenterShootBack::Run()
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
        double fwd = -0.50;
        double rot = 0.00;
        IO.drivebase.Arcade(fwd, rot);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 2:
    {
        IO.drivebase.Stop();
        IO.shooter.Stop();
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    
    default:
        IO.drivebase.Stop();
    }
}