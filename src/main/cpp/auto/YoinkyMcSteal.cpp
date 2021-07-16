#include "auto/YoinkyMcSteal.hpp"

// Name for Smart Dash Chooser
std::string YoinkyMcSteal::GetName()
{
    return "9 - YoinkyMcSteal";
}

// Initialization
// Constructor requires a reference to the robot map
YoinkyMcSteal::YoinkyMcSteal(robotmap &IO) : IO(IO)
{
    m_state = 0;
    IO.drivebase.Stop();
}

YoinkyMcSteal::~YoinkyMcSteal() {}

//State Machine
void YoinkyMcSteal::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    data.filled = false;
    tpCt = 0;
    IO.shooter.Stop();
    IO.RJV.Reset();
}

void YoinkyMcSteal::Init()
{
    units::feet_per_second_t maxLinearVel = 13_fps;
    units::standard_gravity_t maxCentripetalAcc = 0.5_SG;
    units::feet_per_second_squared_t maxLinearAcc = 10_fps_sq;
    // units::meters_per_second_t maxLinearVel = 15_fps;
    // units::standard_gravity_t maxCentripetalAcc = 1_SG;
    // units::meters_per_second_squared_t maxLinearAcc = 14_fps_sq;

    frc::EllipticAccelerationConstraint m_elliptic_constraint{maxCentripetalAcc, maxLinearAcc};

    frc::DifferentialDriveKinematicsConstraint m_kinematic_constraint{IO.drivebase.GetKinematics(), maxLinearVel};

    frc::DifferentialDriveVoltageConstraint m_voltage_constraint{IO.drivebase.GetFeedForward(), IO.drivebase.GetKinematics(), 12.5_V};

    //rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{theChonkster};

    frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc);
cout << "A" << endl;
    config.AddConstraint(m_elliptic_constraint);
    config.AddConstraint(m_voltage_constraint);
    config.AddConstraint(m_kinematic_constraint);

    config.SetReversed(true);

cout << "B" << endl;
    std::vector<frc::Spline<5>::ControlVector> p1;

    {
        io::CSVReader<6> csv("/home/lvuser/deploy/PathWeaver/Paths/McStealA");
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
cout << "C" << endl;
    config.SetReversed(false);
cout << "D" << endl;
    std::vector<frc::Spline<5>::ControlVector> p2;

    {
        io::CSVReader<6> csv("/home/lvuser/deploy/PathWeaver/Paths/McStealB");
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
cout << "E" << endl;
    config.SetReversed(true);
cout << "F" << endl;
    std::vector<frc::Spline<5>::ControlVector> p3;

    {
        io::CSVReader<6> csv("/home/lvuser/deploy/PathWeaver/Paths/McStealC");
        csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
        double x, y, dx, dy, ddx = 0, ddy = 0;
        while (csv.read_row(x, y, dx, dy, ddx, ddy))
        {
            //std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
            p3.push_back({{x, dx, ddx}, {y, dy, ddy}});
        }
    }

    auto t3 = frc::TrajectoryGenerator::GenerateTrajectory(p3, config);
    auto s3 = t3.States();

    config.SetReversed(false);

    std::vector<frc::Spline<5>::ControlVector> p4;

    {
        io::CSVReader<6> csv("/home/lvuser/deploy/PathWeaver/Paths/McStealD");
        csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
        double x, y, dx, dy, ddx = 0, ddy = 0;
        while (csv.read_row(x, y, dx, dy, ddx, ddy))
        {
            //std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
            p4.push_back({{x, dx, ddx}, {y, dy, ddy}});
        }
    }

    auto t4 = frc::TrajectoryGenerator::GenerateTrajectory(p4, config);
    auto s4 = t4.States();
cout << "G" << endl;
    m_trajectory = frc::Trajectory(s1);
    m_trajectorydos = frc::Trajectory(s2);
    m_trajectorytres = frc::Trajectory(s3);
    m_trajectorycuatro = frc::Trajectory(s4);
cout << "H" << endl;
    m_autoTimer.Reset();
    m_autoTimer.Start();

    IO.drivebase.ResetOdometry(m_trajectory.InitialPose());
cout << "I" << endl;
    // for(auto x : m_trajectory.States())
    // {
    //     cout << x.pose.X().value() << "," << x.pose.Y().value() << endl;
    // }
}

// Execute the program
void YoinkyMcSteal::Run()
{
    switch (m_state)
    {
    case 0:
    {
        IO.shooter.SetHoodAngle(17);

        auto reference = m_trajectory.Sample(m_autoTimer.Get());
        //cout << reference.pose.X().value() << "," << reference.pose.Y().value() << endl;

        auto speeds = IO.m_ramsete.Calculate(IO.drivebase.GetPose(), reference);

        IO.drivebase.MeasuredDrive(speeds.vx, speeds.omega);

        //cout << IO.drivebase.GetPose().X().value() << "," << IO.drivebase.GetPose().Y().value() << endl;

        if (m_autoTimer.Get().value() > 0.5)
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
    case 1:
    {
        auto reference = m_trajectorydos.Sample(m_autoTimer.Get());
        //cout << reference.pose.X().value() << "," << reference.pose.Y().value() << endl;

        auto speeds = IO.m_ramsete.Calculate(IO.drivebase.GetPose(), reference);

        IO.drivebase.MeasuredDrive(speeds.vx, speeds.omega);

        //cout << IO.drivebase.GetPose().X().value() << "," << IO.drivebase.GetPose().Y().value() << endl;
        if ((m_autoTimer.Get().value() > 1.0))
            IO.shooter.SetIntake(0.0);
        IO.shooter.SetIndexer(0.0);
        IO.shooter.SetFeeder(0.0);

        if ((m_autoTimer.Get() > m_trajectorydos.TotalTime()))
        {
            NextState();
        }
        break;
    }
    case 2:
    {
        IO.shooter.SetVelocity(4000.0);
        IO.shooter.SetHoodAngle(62.5);
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
    case 3:
    {
        IO.shooter.SetVelocity(4000.0);
        IO.shooter.SetHoodAngle(62.5);
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
    case 4:
    {
        IO.shooter.SetHoodAngle(17);

        auto reference = m_trajectorytres.Sample(m_autoTimer.Get());
        //cout << reference.pose.X().value() << "," << reference.pose.Y().value() << endl;

        auto speeds = IO.m_ramsete.Calculate(IO.drivebase.GetPose(), reference);

        IO.drivebase.MeasuredDrive(speeds.vx, speeds.omega);

        //cout << IO.drivebase.GetPose().X().value() << "," << IO.drivebase.GetPose().Y().value() << endl;

        if (m_autoTimer.Get().value() > 0.5)
        {
            IO.shooter.SetIntake(100.0);
        }
        else
        {
            IO.shooter.SetIntake(0.0);
            IO.shooter.SetIndexer(0.0);
            IO.shooter.SetFeeder(0.0);
        }

        if ((m_autoTimer.Get() > m_trajectorydos.TotalTime()))
        {
            NextState();
        }
        break;
    }
    case 5:
    {
        auto reference = m_trajectorydos.Sample(m_autoTimer.Get());
        //cout << reference.pose.X().value() << "," << reference.pose.Y().value() << endl;

        auto speeds = IO.m_ramsete.Calculate(IO.drivebase.GetPose(), reference);

        IO.drivebase.MeasuredDrive(speeds.vx, speeds.omega);

        //cout << IO.drivebase.GetPose().X().value() << "," << IO.drivebase.GetPose().Y().value() << endl;
        if ((m_autoTimer.Get().value() > 1.0))
            IO.shooter.SetIntake(0.0);
        IO.shooter.SetIndexer(0.0);
        IO.shooter.SetFeeder(0.0);

        if ((m_autoTimer.Get() > m_trajectorydos.TotalTime()))
        {
            NextState();
        }
        break;
    }
    case 6:
    {
        IO.shooter.SetVelocity(4000.0);
        IO.shooter.SetHoodAngle(62.5);
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
        IO.shooter.SetVelocity(4000.0);
        IO.shooter.SetHoodAngle(62.5);
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

void YoinkyMcSteal::UpdateSmartDash()
{
    SmartDashboard::PutNumber("Auto State", m_state);
}