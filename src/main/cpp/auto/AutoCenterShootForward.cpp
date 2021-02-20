#include "auto/AutoCenterShootForward.hpp"

// Name for Smart Dash Chooser
std::string AutoCenterShootForward::GetName()
{
    return "2 - AutoCenterShootForward";
}

// Initialization
// Constructor requires a reference to the robot map
AutoCenterShootForward::AutoCenterShootForward(RobotMap &IO) : IO(IO)
{
    m_state = 0;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    IO.drivebase.Stop();
}

AutoCenterShootForward::~AutoCenterShootForward() {}

//State Machine
void AutoCenterShootForward::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

// Execute the program
void AutoCenterShootForward::Run()
{
    switch (m_state)
    {
    case 0:
    {
        IO.drivebase.Stop();

        data = IO.RJV.Run(IO.RJV.ShotType::Three);
        if (data.filled)
        {
            if (tpCt > 7)
            {
                IO.shooter.SetShooterDistanceThree(data.distance);
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
            //NextState();
        }
        break;
    }
    case 1:
    {
        double fwd = 0.20;
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

void AutoCenterShootForward::UpdateSmartDash()
{
    SmartDashboard::PutNumber("Auto State", m_state);
}