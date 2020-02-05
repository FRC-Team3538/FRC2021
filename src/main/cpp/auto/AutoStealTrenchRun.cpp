#include "auto/AutoStealTrenchRun.hpp"

// Name for Smart Dash Chooser
std::string AutoStealTrenchRun::GetName()
{
    return "4 - AutoStealTrenchRun";
}

// Initialization
// Constructor requires a reference to the robot map
AutoStealTrenchRun::AutoStealTrenchRun(robotmap &IO) : IO(IO)
{
    m_state = 0;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    IO.drivebase.Stop();
}

AutoStealTrenchRun::~AutoStealTrenchRun() { }

//State Machine
void AutoStealTrenchRun::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

// Execute the program
void AutoStealTrenchRun::Run()
{
    switch (m_state)
    {
    case 0:
    {
        IO.shooter.IntakeDeploy();
        IO.shooter.SetShooterDistanceThree(108.0);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 1:
    {
        IO.drivebase.TurnAbs(90.0);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 2:
    {
        IO.drivebase.DriveForward(36.0, 0.15);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 3:
    {
        IO.drivebase.TurnAbs(0.0, 0.15);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
     case 4:
     {
        IO.shooter.SetIntake(0.5);
        IO.drivebase.DriveForward(-180.0, 0.15);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 5:
     {
        IO.shooter.SetIntake(0.0);
        IO.drivebase.DriveForward(-108.0, 0.15);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 6:
    {
        IO.drivebase.TurnAbs(-15.0, 0.15);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 7:
    {
        IO.shooter.SetShooterDistanceTwo(204);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    
    default:
        IO.drivebase.Stop();
    }

    UpdateSmartDash();
}

void AutoStealTrenchRun::UpdateSmartDash()
{
    SmartDashboard::PutNumber("Auto State", m_state);
}