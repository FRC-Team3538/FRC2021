#include "auto/AutoTenBall.hpp"

// Name for Smart Dash Chooser
std::string AutoTenBall::GetName()
{
    return "4 - AutoStealTrenchRun";
}

// Initialization
// Constructor requires a reference to the robot map
AutoTenBall::AutoTenBall(robotmap &IO) : IO(IO)
{
    m_state = 0;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    IO.drivebase.Stop();
}

AutoTenBall::~AutoTenBall() { }

//State Machine
void AutoTenBall::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

// Execute the program
void AutoTenBall::Run()
{
    switch (m_state)
    {
    case 0:
    {
        IO.shooter.IntakeDeploy();
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 1:
    {
        IO.shooter.SetIntake(0.7);
        IO.drivebase.DriveForward(-130.0, 0.15);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 2:
    {
        IO.shooter.SetIntake(0.0);
        IO.drivebase.DriveForward(0.0, 0.15);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 3:
    {
        IO.drivebase.TurnAbs(90.0, 0.15);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
     case 4:
     {
        IO.drivebase.DriveForward(90.0, 0.15);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 5:
     {
        IO.drivebase.TurnAbs(0.0, 0.15);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 6:
    {
        IO.shooter.SetShooterDistanceThree(108.0);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 7:
    {
        IO.drivebase.TurnAbs(90.0, 0.15);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 8:
    {
        IO.drivebase.DriveForward(90.0, 0.15);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 9:
    {
        IO.drivebase.TurnAbs(0.0, 0.15);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 10:
    {
        IO.shooter.SetIntake(0.7);
        IO.drivebase.DriveForward(-258.0, 0.15);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 11:
    {
        IO.shooter.SetIntake(0.0);
        IO.drivebase.DriveForward(0.0, 0.15);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 12:
    {
        IO.drivebase.TurnAbs(-90.0, 0.15);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 13:
    {
        IO.drivebase.DriveForward(90.0, 0.15);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 14:
    {
        IO.drivebase.TurnAbs(0.0, 0.15);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 15:
    {
        IO.shooter.SetShooterDistanceThree(108.0);
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

void AutoTenBall::UpdateSmartDash()
{
    SmartDashboard::PutNumber("Auto State", m_state);
}