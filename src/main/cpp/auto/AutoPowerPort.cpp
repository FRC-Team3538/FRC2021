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

void AutoPowerPort::Init()
{
    wpi::json j;

    frc::Pose2d shoot{0_in, 0_in, 0_deg};
    frc::Pose2d load{-120_in, 0_in, 0_deg};
    frc::TrajectoryConfig config(3_fps, 3_fps_sq);

    config.AddConstraint(m_drivetrain_constraint);

    // NOTE: These must be in meters but Spline<> takes doubles so we ignore typing
    std::vector<frc::Spline<5>::ControlVector> points
    {    
        {
            {1.6197307137707282, 0.7874091564527756, 0}, //x
            {-2.216777217015141, 0.969118961788032, 0} //y
        },
        {
            {2.260041456380678, 0.01730569574621521, 0}, //x
            {-1.2303525594808942, 0.6749221341023793, 0} //y
        }
    };

    // Go to Load
    m_trajectory_rev = frc::TrajectoryGenerator::GenerateTrajectory(
        points,
        config);

    // Go To Shoot
    config.SetReversed(true);
    m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        shoot,
        {},
        load,
        config);
}

// Execute the program
void AutoPowerPort::Run()
{

    switch (m_state)
    {
    case 0:
    {
        IO.shooter.IntakeDeploy();
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

        if (m_autoTimer.Get().value() > 6.0)
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
        auto elapsed = m_autoTimer.Get();
        auto reference = m_trajectory.Sample(elapsed);

        wpi::json j;
        frc::to_json(j, reference);
        auto reference_json = j.dump(0);

        frc::SmartDashboard::PutString("Reference", reference_json);

        auto speeds = m_ramsete.Calculate(IO.drivebase.GetPose(), reference);
        IO.drivebase.MeasuredDrive(speeds.vx, speeds.omega);

        if (elapsed > (m_trajectory.TotalTime()))
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

        if (m_autoTimer.Get().value() > SmartDashboard::GetNumber("PowerPort Delay", 7.0))
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
        auto elapsed = m_autoTimer.Get();
        auto reference = m_trajectory_rev.Sample(elapsed);

        auto speeds = m_ramsete.Calculate(IO.drivebase.GetPose(), reference);
        IO.drivebase.MeasuredDrive(speeds.vx, speeds.omega);

        if (elapsed > (m_trajectory_rev.TotalTime()))
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