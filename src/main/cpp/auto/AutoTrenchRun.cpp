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
    setstate = 0;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    IO.drivebase.Stop();
}

AutoTrenchRun::~AutoTrenchRun() {}

//State Machine
void AutoTrenchRun::NextState()
{
    setstate++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    data.filled = false;
    tpCt = 0;
    IO.shooter.StopShooter();
    IO.RJV.Reset();
}

// Execute the program
void AutoTrenchRun::Run()
{
    switch (setstate)
    {
    case 0:
    {
        IO.drivebase.Stop();

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
    case 3:
    {
        IO.drivebase.DriveForward(-108);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }

    case 4:
    {
        IO.drivebase.TurnAbs(0);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 5:
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
        IO.shooter.Stop();
    }

    UpdateSmartDash();
}

void AutoTrenchRun::UpdateSmartDash()
{
    SmartDashboard::PutNumber("Auto State", setstate);
}