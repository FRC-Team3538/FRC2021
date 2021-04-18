#include "auto/AutoTrenchRun.hpp"

// Name for Smart Dash Chooser
std::string AutoTrenchRun::GetName()
{
    return "2 - AutoTrenchRun";
}

// Initialization
// Constructor requires a reference to the robot map
AutoTrenchRun::AutoTrenchRun(RobotMap &IO) : IO(IO)
{
    m_state = 0;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    IO.drivebase.Stop();
    IO.drivebase.ResetGyro();
}

AutoTrenchRun::~AutoTrenchRun() {}

//State Machine
void AutoTrenchRun::NextState()
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
void AutoTrenchRun::Run()
{
    switch (m_state)
    {
    case 0:
    {
        const double shootSpeedz = 3000.0;

        IO.drivebase.Stop();
        IO.shooter.IntakeDeploy();
        IO.shooter.SetVelocity(shootSpeedz);
        IO.shooter.SetHoodAngle(60.5);
        if (abs(shootSpeedz - IO.shooter.GetVelocity()) < 100.0)
        {
            IO.shooter.SetIndexer(100.0);
            IO.shooter.SetFeeder(100.0);
        }

        if (m_autoTimer.Get() > 3.5)
        {
            NextState();
        }

        // IO.shooter.IntakeDeploy();
        // IO.drivebase.Stop();
        // data = IO.RJV.Run(IO.RJV.ShotType::Two);
        // if (data.filled)
        // {
        //     if (tpCt > 5)
        //     {
        //         IO.shooter.SetShooterDistanceTwo(data.distance);
        //         IO.drivebase.Arcade(0.0, 0.0);
        //     }
        //     else
        //     {
        //         IO.drivebase.TurnRel(data.angle, 0.5) ? tpCt++ : tpCt = 0;
        //         IO.shooter.SetVelocity(1000.0);
        //     }
        // }

        break;
    }
    case 1:
    {
        IO.shooter.Stop();
        IO.shooter.SetIndexer(0.0);
        IO.shooter.SetFeeder(0.0);

        IO.drivebase.TurnAbs(-15.0, 0.20);
        IO.shooter.IntakeDeploy();
        if (IO.drivebase.GetGyroHeading() < -10)
        {
            NextState();
        }
        break;
    }
    case 2:
    {
        IO.drivebase.DriveForward(-155.0, 0.2);
        IO.shooter.SetIntake(0.5);
        IO.shooter.SetIndexer(0.5);
        if (IO.drivebase.GetEncoderPosition() < -150.0)
        {
            NextState();
        }
        break;
    }
    case 3:
    {
        IO.drivebase.DriveForward(-108.0, 0.20);
        if (IO.drivebase.GetEncoderPosition() > -113.0)
        {
            NextState();
        }
        break;
    }

    case 4:
    {
        IO.drivebase.TurnAbs(-5.0, 0.20);
        if (IO.drivebase.GetGyroHeading() > -10.0)
        {
            NextState();
        }
        break;
    }
    case 5:
    {

        IO.drivebase.Stop();
        IO.shooter.IntakeDeploy();
        IO.shooter.SetVelocity(3500.0);
        IO.shooter.SetHoodAngle(64.5);
        if ((abs(3500.0 - IO.shooter.GetVelocity()) < 150.0))
        {
            IO.shooter.SetIndexer(100.0);
            IO.shooter.SetFeeder(100.0);
        }

        if (m_autoTimer.Get() > 3.5)
        {
            NextState();
        }
        break;
    }

    default:
        IO.drivebase.Stop();
        IO.shooter.Stop();
        IO.shooter.SetIndexer(0.0);
        IO.shooter.SetFeeder(0.0);
    }

    UpdateSmartDash();
}

void AutoTrenchRun::UpdateSmartDash()
{
    SmartDashboard::PutNumber("Auto State", m_state);
}