#include "auto/AutoPowerPort.hpp"

// Name for Smart Dash Chooser
std::string AutoPowerPort::GetName()
{
    return "9 - AutoPowerPort";
}

// Initialization
// Constructor requires a reference to the robot map
AutoPowerPort::AutoPowerPort(robotmap &IO) : IO(IO)
{
    m_state = 0;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    IO.drivebase.Stop();
}

AutoPowerPort::~AutoPowerPort() {}

//State Machine
void AutoPowerPort::NextState()
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
void AutoPowerPort::Run()
{
    switch (m_state)
    {
    case 0:
    {
        //IO.shooter.IntakeDeploy();
        data = IO.RJV.Run(0.0);

        IO.shooter.SetVelocity(3000.0);
        IO.shooter.SetHoodAngle(59.0);

        double dVel = (IO.shooter.GetVelocity() - prevRPM) / 0.02;

        if (data.filled)
        {

            if ((abs(3000.0 - IO.shooter.GetVelocity()) < 100.0) && (abs(IO.shooter.GetHoodAngle() - 59.0) < 1.5) && IO.drivebase.VisionAim(0.0, data.angle, 0.3))
            {
                IO.shooter.SetIntake(100.0);
                IO.shooter.SetIndexer(100.0);
                IO.shooter.SetFeeder(100.0);
            }
            else
            {
                IO.shooter.SetFeeder(0.0);
            }
        }

        if((dVel < -400.0) && !shotCounterOS)
        {
            ++shotCounter;
            shotCounterOS = true;
        }
        else if(dVel > 250.0)
        {
            shotCounterOS = false;
        }

        prevRPM = IO.shooter.GetVelocity();
        if (shotCounter >= 3)
            NextState();

        if (m_autoTimer.Get() > 6.0)
        {
            NextState();
        }
        break;
    }
    case 1:
    {
        IO.shooter.Stop();
        IO.shooter.SetFeeder(0.0);
        IO.shooter.SetIndexer(0.0);
        IO.shooter.SetIntake(0.0);
        IO.drivebase.DriveForward(-165.0, 0.75);
        if (IO.drivebase.GetEncoderPosition() <= -162.0)
        {
            NextState();
        }
        break;
    }
    case 2:
    {
        IO.drivebase.Stop();
        IO.shooter.SetIndexer(1.0);
        IO.shooter.SetIntake(1.0);

        if (m_autoTimer.Get() > SmartDashboard::GetNumber("PowerPort Delay", 7.0))
        {
            NextState();
        }
        break;
    }
    case 3:
    {
        IO.shooter.Stop();
        IO.shooter.SetFeeder(0.0);
        IO.shooter.SetIndexer(0.0);
        IO.shooter.SetIntake(0.0);
        IO.drivebase.DriveForward(0.0, 0.75);
        if (IO.drivebase.GetEncoderPosition() >= -4.0)
        {
            NextState();
        }
        break;
    }
    default:
    {
        IO.drivebase.Stop();
        m_state = 0;
    }
    }

    UpdateSmartDash();
}

void AutoPowerPort::UpdateSmartDash()
{
    SmartDashboard::PutNumber("Auto State", m_state);
    SmartDashboard::PutNumber("PowerPort Delay", SmartDashboard::GetNumber("PowerPort Delay", 5.0));
}