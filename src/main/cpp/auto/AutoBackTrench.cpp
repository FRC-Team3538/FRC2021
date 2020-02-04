#include "auto/AutoBackTrench.hpp"

// Name for Smart Dash Chooser
std::string AutoBackTrench::GetName()
{
    return "3 - AutoBackTrench";
}

// Initialization
// Constructor requires a reference to the robot map
AutoBackTrench::AutoBackTrench(robotmap &IO) : IO(IO)
{
    m_state = 0;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    IO.drivebase.Stop();
}

AutoBackTrench::~AutoBackTrench() {}

//State Machine
void AutoBackTrench::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    data.filled = false;
    tpCt = 0;
    IO.shooter.StopShooter();
    IO.RJV.Reset();
}

// Execute the program
void AutoBackTrench::Run()
{
    switch (m_state)
    {
    case 0:
    {
        IO.shooter.IntakeDeploy();

        data = IO.RJV.Run(IO.RJV.ShotType::Three);
        if (data.filled)
        {
            if (tpCt > 10)
            {
                IO.shooter.SetShooterDistanceThree(data.distance);
                IO.drivebase.Arcade(0.0, 0.0);
            }
            else
            {
                IO.drivebase.TurnRel(data.angle, 0.5) ? tpCt++ : tpCt = 0;
                IO.shooter.SetVelocity(1500.0);
            }
        }

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
        IO.drivebase.DriveForward(36.0);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 3:
    {
        IO.drivebase.TurnAbs(0.0);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 4:
    {
        IO.shooter.SetIntake(0.5);
        IO.drivebase.DriveForward(-180.0);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 5:
    {
        IO.shooter.SetIntake(0.0);
        IO.drivebase.DriveForward(-108.0);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 6:
    {
        IO.drivebase.TurnAbs(-15.0);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 7:
    {
        data = IO.RJV.Run(IO.RJV.ShotType::Two);
        if (data.filled)
        {
            if (tpCt > 5)
            {
                IO.shooter.SetShooterDistanceTwo(data.distance);
                IO.drivebase.Arcade(0.0, 0.0);
            }
            else
            {
                IO.drivebase.TurnRel(data.angle, 0.5) ? tpCt++ : tpCt = 0;
                IO.shooter.SetVelocity(1000.0);
            }
        }

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

void AutoBackTrench::UpdateSmartDash()
{
    SmartDashboard::PutNumber("Auto State", m_state);
}