#include "auto/AutoIntAcc.hpp"

// Name for Smart Dash Chooser
std::string AutoIntAcc::GetName()
{
    return "9 - AutoIntAcc";
}

// Initialization
// Constructor requires a reference to the robot map
AutoIntAcc::AutoIntAcc(robotmap &IO) : IO(IO)
{
    m_state = 0;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    IO.drivebase.Stop();
}

AutoIntAcc::~AutoIntAcc() {}

//State Machine
void AutoIntAcc::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    data.filled = false;
    tpCt = 0;
    IO.shooter.Stop();
    IO.RJV.Reset();
    shotCounter = 0;
}

// Execute the program
void AutoIntAcc::Run()
{
    switch (m_state)
    {
    case 0:
    {
        //IO.shooter.IntakeDeploy();
        data = IO.RJV.Run(0.0);

        IO.shooter.SetVelocity(2780.0);
        IO.shooter.SetHoodAngle(51.0);

        if (data.filled)
        {

            if ((abs(2750.0 - IO.shooter.GetVelocity()) < 80.0) && (abs(IO.shooter.GetHoodAngle() - 51.0) < 1.5) && IO.drivebase.VisionAim(0.0, data.angle, 0.3))
            {
                IO.shooter.SetIntake(100.0);
                IO.shooter.SetIndexer(100.0);
                IO.shooter.SetFeeder(100.0);
                shotCounterOS = false;
            }
            else
            {
                if (!shotCounterOS && !((abs(2750.0 - IO.shooter.GetVelocity()) < 80.0) && (abs(IO.shooter.GetHoodAngle() - 51.0) < 1.5)))
                {
                    ++shotCounter;
                    shotCounterOS = true;
                }
                IO.shooter.SetFeeder(0.0);
            }
        }
        if (shotCounter >= 4)
            NextState();

        if (m_autoTimer.Get() > 3.0)
        {
            NextState();
        }
        break;
    }
    case 1:
    {
        IO.RJV.SetPipeline(1.0);
        IO.shooter.Stop();
        IO.shooter.SetFeeder(0.0);
        IO.shooter.SetIndexer(0.0);
        IO.shooter.SetIntake(0.0);
        IO.drivebase.DriveForward(-210.0, 0.2);
        if (IO.drivebase.GetEncoderPosition() <= -210.0)
        {
            NextState();
        }
        break;
    }
    case 2:
    {
        IO.shooter.SetIndexer(1.0);
        IO.shooter.SetIntake(1.0);

        if (m_autoTimer.Get() > 10.0)
        {
            NextState();
        }
        break;
    }
    case 3:
    {
        IO.shooter.Stop();
        IO.drivebase.DriveForward(-180, 0.2);
        IO.shooter.SetIntake(0.3);
        IO.shooter.SetIndexer(0.3);
        if (IO.drivebase.GetEncoderPosition() > -180.0)
        {
            NextState();
        }
        break;
    }
    case 4:
    {
        data = IO.RJV.Run(1.0);

        IO.shooter.SetVelocity(4000.0);
        IO.shooter.SetHoodAngle(66.6);

        if (data.filled)
        {

            if ((abs(4000.0 - IO.shooter.GetVelocity()) < 80.0) && (abs(IO.shooter.GetHoodAngle() - 66.6) < 1.5) && IO.drivebase.VisionAim(0.0, data.angle, 0.3))
            {
                IO.shooter.SetIntake(100.0);
                IO.shooter.SetIndexer(100.0);
                IO.shooter.SetFeeder(100.0);
                shotCounterOS = false;
            }
            else
            {
                if (!shotCounterOS && !((abs(4000.0 - IO.shooter.GetVelocity()) < 80.0) && (abs(IO.shooter.GetHoodAngle() - 66.6) < 1.5)))
                {
                    ++shotCounter;
                    shotCounterOS = true;
                }
                IO.shooter.SetFeeder(0.0);
            }
        }
        if (shotCounter >= 4)
            NextState();

        if (m_autoTimer.Get() > 3.0)
        {
            NextState();
        }
        break;
    }
    case 5:
    {
        IO.RJV.SetPipeline(0.0);
        IO.shooter.Stop();
        IO.shooter.SetFeeder(0.0);
        IO.shooter.SetIndexer(0.0);
        IO.shooter.SetIntake(0.0);
        IO.drivebase.DriveForward(-210.0, 0.2);
        if (IO.drivebase.GetEncoderPosition() <= -210.0)
        {
            NextState();
        }
        break;
    }
    case 6:
    {
        IO.shooter.SetIndexer(1.0);
        IO.shooter.SetIntake(1.0);

        if (m_autoTimer.Get() > 10.0)
        {
            NextState();
        }
        break;
    }
    case 7:
    {
        IO.shooter.Stop();
        IO.drivebase.DriveForward(-120.0, 0.2);
        IO.shooter.SetIntake(0.3);
        IO.shooter.SetIndexer(0.3);
        if (IO.drivebase.GetEncoderPosition() > -120.0)
        {
            NextState();
        }
        break;
    }
    case 8:
    {
        data = IO.RJV.Run(0.0);

        IO.shooter.SetVelocity(3500.0);
        IO.shooter.SetHoodAngle(64.0);

        if (data.filled)
        {

            if ((abs(3500.0 - IO.shooter.GetVelocity()) < 80.0) && (abs(IO.shooter.GetHoodAngle() - 64.0) < 1.5) && IO.drivebase.VisionAim(0.0, data.angle, 0.3))
            {
                IO.shooter.SetIntake(100.0);
                IO.shooter.SetIndexer(100.0);
                IO.shooter.SetFeeder(100.0);
                shotCounterOS = false;
            }
            else
            {
                if (!shotCounterOS && !((abs(3500.0 - IO.shooter.GetVelocity()) < 80.0) && (abs(IO.shooter.GetHoodAngle() - 64.0) < 1.5)))
                {
                    ++shotCounter;
                    shotCounterOS = true;
                }
                IO.shooter.SetFeeder(0.0);
            }
        }
        if (shotCounter >= 4)
            NextState();

        if (m_autoTimer.Get() > 3.0)
        {
            NextState();
        }

        break;
    }
    case 9:
    {
        IO.shooter.Stop();
        IO.shooter.SetFeeder(0.0);
        IO.shooter.SetIndexer(0.0);
        IO.shooter.SetIntake(0.0);
        IO.drivebase.DriveForward(-210.0, 0.2);
        if (IO.drivebase.GetEncoderPosition() <= -210.0)
        {
            NextState();
        }
        break;
    }
    case 10:
    {
        IO.shooter.SetIndexer(1.0);
        IO.shooter.SetIntake(1.0);

        if (m_autoTimer.Get() > 10.0)
        {
            NextState();
        }
        break;
    }
    case 11:
    {
        IO.shooter.Stop();
        IO.drivebase.DriveForward(-60.0, 0.2);
        IO.shooter.SetIntake(0.3);
        IO.shooter.SetIndexer(0.3);
        if (IO.drivebase.GetEncoderPosition() > -60.0)
        {
            NextState();
        }
        break;
    }
    case 12:
    {
        data = IO.RJV.Run(0.0);

        IO.shooter.SetVelocity(3000.0);
        IO.shooter.SetHoodAngle(59.0);

        if (data.filled)
        {

            if ((abs(3000.0 - IO.shooter.GetVelocity()) < 80.0) && (abs(IO.shooter.GetHoodAngle() - 59.0) < 1.5) && IO.drivebase.VisionAim(0.0, data.angle, 0.3))
            {
                IO.shooter.SetIntake(100.0);
                IO.shooter.SetIndexer(100.0);
                IO.shooter.SetFeeder(100.0);
                shotCounterOS = false;
            }
            else
            {
                if (!shotCounterOS && !((abs(3000.0 - IO.shooter.GetVelocity()) < 80.0) && (abs(IO.shooter.GetHoodAngle() - 59.0) < 1.5)))
                {
                    ++shotCounter;
                    shotCounterOS = true;
                }
                IO.shooter.SetFeeder(0.0);
            }
        }
        if (shotCounter >= 4)
            NextState();

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

void AutoIntAcc::UpdateSmartDash()
{
    SmartDashboard::PutNumber("Auto State", m_state);
}