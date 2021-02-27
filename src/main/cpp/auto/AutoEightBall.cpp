#include "auto/AutoEightBall.hpp"

// Name for Smart Dash Chooser
std::string AutoEightBall::GetName()
{
    return "7 - AutoEightBall";
}

// Initialization
// Constructor requires a reference to the robot map
AutoEightBall::AutoEightBall(robotmap &IO) : IO(IO)
{
    m_state = 0;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    //IO.drivebase.Stop(); 
}

AutoEightBall::~AutoEightBall() {}

//State Machine
void AutoEightBall::NextState()
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
void AutoEightBall::Run()
{
    switch (m_state)
    {
    case 0:
    {
        IO.RJV.SetPipeline(0.0);
        IO.drivebase.Stop();
        IO.shooter.IntakeDeploy();
        IO.shooter.SetVelocity(3000.0);
        IO.shooter.SetHoodAngle(60.5);
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
        IO.shooter.Stop();
        IO.shooter.SetFeeder(0.0);
        IO.shooter.SetIntake(0.0);
        IO.drivebase.TurnAbs(3.2, 0.5);
        IO.shooter.SetHoodAngle(0.0);

        if (abs(IO.drivebase.GetGyroHeading() - 3.2) > 1.0)
        {
            m_autoTimer.Reset();
        }

        if (m_autoTimer.Get() > 0.3)
        {
            NextState();
        }
        break;
    }
    case 2:
    {
        IO.shooter.SetHoodAngle(0.0);
        IO.shooter.Stop();
        IO.shooter.SetFeeder(0.0);
        IO.drivebase.DriveForward(-109.0, 0.6);
        IO.shooter.SetIntake(0.5);
        IO.shooter.SetIndexer(0.5);
        if (IO.drivebase.GetEncoderPositionLeft() < -109.0)
        {
            NextState();
        }
        break;
    }
    case 3:
    {
        IO.shooter.SetHoodAngle(0.0);
        IO.drivebase.Stop();
        IO.drivebase.TurnAbs(-10.0, 0.20);

        if (abs(IO.drivebase.GetGyroHeading() - (-10.0)) > 1.3)
        {
            m_autoTimer.Reset();
        }

        if (m_autoTimer.Get() > 0.3)
        {
            NextState();
        }

        break;
    }
    case 4:
    {
        IO.drivebase.DriveForward(-227.0, 0.8);
        IO.shooter.SetIntake(0.5);
        IO.shooter.SetIndexer(0.5);
        if (IO.drivebase.GetEncoderPositionLeft() < -221.0)
        {
            NextState();
        }
        break;
    }
    case 5:
    {
        IO.drivebase.DriveForward(-92.0, 0.8);
        IO.shooter.SetIntake(0.0);
        IO.shooter.SetIndexer(0.0);
        IO.shooter.SetVelocity(3000.0);
        IO.shooter.SetHoodAngle(60.5);
        if (IO.drivebase.GetEncoderPositionLeft() > -92.0)
        {
            NextState();
        }
        break;
    }
    case 6:
    {
        IO.shooter.SetVelocity(3000.0);
        IO.shooter.SetHoodAngle(60.5);
        data = IO.RJV.Run(IO.RJV.Pipe::TwoClose);
        if (data.filled)
        {
            IO.drivebase.VisionAim(0, data.angle, 0.6);
            if (abs(data.angle) < 0.6)
            {
                NextState();
            }
        }
        break;
    }
    case 7:
    {
        IO.shooter.SetVelocity(3000.0);
        IO.shooter.SetHoodAngle(60.5);
        if ((abs(3000.0 - IO.shooter.GetVelocity()) < 100.0))
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

    default:
        IO.drivebase.Stop();
    }

    UpdateSmartDash();
}

void AutoEightBall::UpdateSmartDash()
{
    SmartDashboard::PutNumber("Auto State", m_state);
}