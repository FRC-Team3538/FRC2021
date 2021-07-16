#include "auto/AutoShootInit.hpp"

// Name for Smart Dash Chooser
std::string AutoShootInit::GetName()
{
    return "7 - AutoShootInit";
}

// Initialization
// Constructor requires a reference to the robot map
AutoShootInit::AutoShootInit(robotmap &IO) : IO(IO)
{
    m_state = 0;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    IO.drivebase.Stop();
}

AutoShootInit::~AutoShootInit() {}

//State Machine
void AutoShootInit::NextState()
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
void AutoShootInit::Run()
{
    switch (m_state)
    {
    case 0:
    {
        IO.RJV.SetPipeline(0.0);
        IO.drivebase.Stop();
        IO.shooter.IntakeDeploy();
        IO.shooter.SetVelocity(3000.0);
        IO.shooter.SetHoodAngle(58.25);
        if ((abs(3000.0 - IO.shooter.GetVelocity()) < 200.0))
        {
            IO.shooter.SetIntake(100.0);
            IO.shooter.SetIndexer(100.0);
            IO.shooter.SetFeeder(100.0);
        }

        if (m_autoTimer.Get() > 3.0)
        {
            NextState();
        }

        break;
    }
    case 1:
    {
        IO.shooter.SetHoodAngle(0.0);
        IO.shooter.Stop();
        IO.shooter.SetFeeder(0.0);
        IO.drivebase.DriveForward(-40.0, 0.6);
        IO.shooter.SetIntake(0.5);
        IO.shooter.SetIndexer(0.5);
        if (IO.drivebase.GetEncoderPositionLeft() < -40.0)
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

void AutoShootInit::UpdateSmartDash()
{
    SmartDashboard::PutNumber("Auto State", m_state);
}