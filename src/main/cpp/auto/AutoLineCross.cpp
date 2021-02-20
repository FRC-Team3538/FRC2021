#include "auto/AutoLineCross.hpp"

// Name for Smart Dash Chooser
std::string AutoLineCross::GetName()
{
    return "1 - AutoLineCross";
}

// Initialization
// Constructor requires a reference to the robot map
AutoLineCross::AutoLineCross(RobotMap &IO) : IO(IO)
{
    m_state = 0;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    IO.drivebase.Stop();
}

AutoLineCross::~AutoLineCross() {}

//State Machine
void AutoLineCross::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

// Execute the program
void AutoLineCross::Run()
{
    switch (m_state)
    {
    case 0:
    {
        IO.drivebase.TurnAbs(3.2, 0.5);
        IO.shooter.SetHoodAngle(0.0);

        if (abs(IO.drivebase.GetGyroHeading() - (3.2)) > 1.0)
        {
            m_autoTimer.Reset();
        }

        if (m_autoTimer.Get() > 0.3)
        {
            NextState();
        }
        break;
    }
    // case 0:
    // {
    //     double fwd = 0.20;
    //     double rot = 0.00;
    //     IO.drivebase.Arcade(fwd, rot);
    //     if (m_autoTimer.Get() > 2.0)
    //     {
    //         NextState();
    //     }
    //     break;
    // }
    // case 1:
    // {
    //     IO.drivebase.Stop();
    //     if (m_autoTimer.Get() > 2.0)
    //     {
    //         NextState();
    //     }
    //     break;
    // }
    default:
        IO.drivebase.Stop();
    }

    UpdateSmartDash();
}

void AutoLineCross::UpdateSmartDash()
{
    SmartDashboard::PutNumber("Auto State", m_state);
}