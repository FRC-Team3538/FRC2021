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

AutoCenterShootBack::~AutoCenterShootBack() {}

//State Machine
void AutoCenterShootBack::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    data.filled = false;
    tpCt = 0;
    IO.shooter.Stop();
    IO.RJV.Reset();
}

// Execute the program
void AutoCenterShootBack::Run()
{
    switch (m_state)
    {
    case 0:
    {
        IO.drivebase.Stop();
        IO.shooter.IntakeDeploy();
        IO.shooter.SetVelocity(3000.0);
        IO.shooter.SetHoodAngle(58.0);
        // data = IO.RJV.Run(IO.RJV.Pipe::ThreeFar);
        // if (data.filled)
        // {
        //     if (tpCt > 4)
        //     {
        //         IO.shooter.SetShooterDistanceTwo(data.distance);
        //         IO.shooter.SetIndexer(1.0);
        //         IO.drivebase.Arcade(0.0, 0.0);
        //     }
        //     else
        //     {
        //         IO.drivebase.TurnRel(data.angle, 0.5) ? tpCt++ : tpCt = 0;
        //         IO.shooter.SetVelocity(3000.0);
        //     }
        // }
        if (m_autoTimer.Get() > 1.0)
        {
            IO.shooter.SetIndexer(100.0);
            IO.shooter.SetFeeder(100.0);
        }
        
        if (m_autoTimer.Get() > 4.0)
        {
            NextState();
        }
        break;
    }
    case 1:
    {
        IO.shooter.Stop();
        IO.shooter.SetIndexer(0.0);
        IO.shooter.SetFeeder(0.0);
        double fwd = -0.15;
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

    UpdateSmartDash();
}

void AutoCenterShootBack::UpdateSmartDash()
{
    SmartDashboard::PutNumber("Auto State", m_state);
}