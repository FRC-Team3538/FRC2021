#include "auto/TrenchRun.hpp"

// Name for Smart Dash Chooser
std::string TrenchRun::GetName()
{
    return "9 - TrenchRun";
}

// Initialization
// Constructor requires a reference to the robot map
TrenchRun::TrenchRun(robotmap &IO) : IO(IO)
{
    m_state = 0;
    IO.drivebase.Stop();
}

TrenchRun::~TrenchRun() {}

//State Machine
void TrenchRun::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    data.filled = false;
    tpCt = 0;
    IO.shooter.Stop();
    IO.RJV.Reset();
}

void TrenchRun::Init()
{
    units::feet_per_second_t maxLinearVel = 7_fps;
    units::standard_gravity_t maxCentripetalAcc = 0.375_SG;
    units::feet_per_second_squared_t maxLinearAcc = 11_fps_sq;
    // units::meters_per_second_t maxLinearVel = 15_fps;
    // units::standard_gravity_t maxCentripetalAcc = 1_SG;
    // units::meters_per_second_squared_t maxLinearAcc = 14_fps_sq;

    frc::EllipticAccelerationConstraint m_elliptic_constraint{maxCentripetalAcc, maxLinearAcc};

    frc::DifferentialDriveKinematicsConstraint m_kinematic_constraint{IO.drivebase.GetKinematics(), maxLinearVel};

    frc::DifferentialDriveVoltageConstraint m_voltage_constraint{IO.drivebase.GetFeedForward(), IO.drivebase.GetKinematics(), 12.5_V};

    //rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{theChonkster};

    frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc);

    config.AddConstraint(m_elliptic_constraint);
    config.AddConstraint(m_voltage_constraint);
    config.AddConstraint(m_kinematic_constraint);

    config.SetReversed(true);

    std::vector<frc::Spline<5>::ControlVector> p1;

    {
        io::CSVReader<6> csv("/home/lvuser/deploy/PathWeaver/Paths/Trench2A");
        csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
        double x, y, dx, dy, ddx = 0, ddy = 0;
        while (csv.read_row(x, y, dx, dy, ddx, ddy))
        {
            //std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
            p1.push_back({{x, dx, ddx}, {y, dy, ddy}});
        }
    }

    auto t1 = frc::TrajectoryGenerator::GenerateTrajectory(p1, config);
    auto s1 = t1.States();

    config.SetReversed(false);

    std::vector<frc::Spline<5>::ControlVector> p2;

    {
        io::CSVReader<6> csv("/home/lvuser/deploy/PathWeaver/Paths/Trench2B");
        csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
        double x, y, dx, dy, ddx = 0, ddy = 0;
        while (csv.read_row(x, y, dx, dy, ddx, ddy))
        {
            //std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
            p2.push_back({{x, dx, ddx}, {y, dy, ddy}});
        }
    }

    auto t2 = frc::TrajectoryGenerator::GenerateTrajectory(p2, config);
    auto s2 = t2.States();

    m_trajectory = t1;//frc::Trajectory(s1);
    m_trajectorydos = t2;//frc::Trajectory(s2);

    m_autoTimer.Reset();
    m_autoTimer.Start();

    IO.drivebase.ResetOdometry(m_trajectory.InitialPose());

    // for(auto x : m_trajectory.States())
    // {
    //     cout << x.pose.X().value() << "," << x.pose.Y().value() << endl;
    // }
}

// Execute the program
void TrenchRun::Run()
{
    switch (m_state)
    {
    case 0:
    {
        IO.shooter.IntakeDeploy();
        IO.shooter.SetVelocity(3000.0);
        IO.shooter.SetHoodAngle(54.5);

        if ((abs(3000.0 - IO.shooter.GetVelocity()) < 150.0) && (abs(IO.shooter.GetHoodAngle() - 53.5) < 1))
        {
            IO.shooter.SetIntake(100.0);
            IO.shooter.SetIndexer(100.0);
            IO.shooter.SetFeeder(100.0);
        }

        if (m_autoTimer.Get().value() > 4.0)
        {
            NextState();
        }

        // if ((abs(3000.0 - IO.shooter.GetVelocity()) < 150.0) && (abs(IO.shooter.GetHoodAngle() - 58.25) < 1.5))
        // {
        //     IO.shooter.SetIntake(100.0);
        //     IO.shooter.SetIndexer(100.0);
        //     IO.shooter.SetFeeder(100.0);
        //     shotCounterOS = false;
        // }
        // else
        // {
        //     if (!shotCounterOS && !((abs(3000.0 - IO.shooter.GetVelocity()) < 150.0) && (abs(IO.shooter.GetHoodAngle() - 58.25) < 1.5)))
        //     {
        //         ++shotCounter;
        //         shotCounterOS = true;
        //     }
        //     IO.shooter.SetFeeder(0.0);
        // }
        break;
    }
    case 1:
    {
        IO.shooter.SetHoodAngle(17);

        auto reference = m_trajectory.Sample(m_autoTimer.Get());
        //cout << reference.pose.X().value() << "," << reference.pose.Y().value() << endl;

        auto speeds = IO.m_ramsete.Calculate(IO.drivebase.GetPose(), reference);

        IO.drivebase.MeasuredDrive(speeds.vx, speeds.omega);

        //cout << IO.drivebase.GetPose().X().value() << "," << IO.drivebase.GetPose().Y().value() << endl;

        if (m_autoTimer.Get().value() > 0.25)
        {
            IO.shooter.SetIntake(100.0);
        }
        else
        {
            IO.shooter.SetIntake(0.0);
            IO.shooter.SetIndexer(0.0);
            IO.shooter.SetFeeder(0.0);
        }

        if ((m_autoTimer.Get() > m_trajectory.TotalTime()))
        {
            NextState();
        }
        break;
    }
    case 2:
    {
        auto reference = m_trajectorydos.Sample(m_autoTimer.Get());
        //cout << reference.pose.X().value() << "," << reference.pose.Y().value() << endl;

        auto speeds = IO.m_ramsete.Calculate(IO.drivebase.GetPose(), reference);

        IO.drivebase.MeasuredDrive(speeds.vx, speeds.omega);

        //cout << IO.drivebase.GetPose().X().value() << "," << IO.drivebase.GetPose().Y().value() << endl;

        IO.shooter.SetIntake(0.0);
        IO.shooter.SetIndexer(0.0);
        IO.shooter.SetFeeder(0.0);

        if ((m_autoTimer.Get() > m_trajectorydos.TotalTime()))
        {
            NextState();
        }
        break;
    }
    case 3:
    {
        IO.shooter.SetVelocity(4000.0);
        IO.shooter.SetHoodAngle(62);
        data = IO.RJV.Run(IO.RJV.Pipe::TwoClose);
        if (data.filled)
        {
            IO.drivebase.VisionAim(0, data.angle, 0.6);
            if (abs(data.angle) < 0.6)
            {
                NextState();
            }
        }
        // if(m_autoTimer.Get().value() > 3.0)
        // {
        //     NextState();
        // }
        break;
    }
    case 4:
    {
        IO.shooter.SetVelocity(4000.0);
        IO.shooter.SetHoodAngle(62);
        if ((abs(4000.0 - IO.shooter.GetVelocity()) < 150.0))
        {
            IO.shooter.SetIntake(100.0);
            IO.shooter.SetIndexer(100.0);
            IO.shooter.SetFeeder(100.0);
        }
        if (m_autoTimer.Get().value() > 3.0)
        {
            NextState();
        }
        break;
    }
    default:
    {
        IO.drivebase.Stop();
        IO.shooter.Stop();
        IO.shooter.SetFeeder(0.0);
        IO.shooter.SetIndexer(0.0);
        IO.shooter.SetIntake(0.0);
    }
    }

    UpdateSmartDash();
}

void TrenchRun::UpdateSmartDash()
{
    SmartDashboard::PutNumber("Auto State", m_state);
}