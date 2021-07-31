#include "auto/BigBalleros.hpp"

// Name for Smart Dash Chooser
std::string BigBalleros::GetName()
{
    return "9 - BigBalleros";
}

// Initialization
// Constructor requires a reference to the robot map
BigBalleros::BigBalleros(robotmap &IO) : IO(IO)
{
    m_state = 0;
    IO.drivebase.Stop();
}

BigBalleros::~BigBalleros() {}

//State Machine
void BigBalleros::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    IO.shooter.Stop();
    IO.RJV.Reset();
}

void BigBalleros::Init()
{
    units::feet_per_second_t maxLinearVel = 13_fps;
    units::standard_gravity_t maxCentripetalAcc = 0.25_SG;
    units::feet_per_second_squared_t maxLinearAcc = 10_fps_sq;
    // units::meters_per_second_t maxLinearVel = 15_fps;
    // units::standard_gravity_t maxCentripetalAcc = 1_SG;
    // units::meters_per_second_squared_t maxLinearAcc = 14_fps_sq;

    frc::EllipticAccelerationConstraint m_elliptic_constraint{maxCentripetalAcc, maxLinearAcc};

    frc::DifferentialDriveKinematicsConstraint m_kinematic_constraint{IO.drivebase.GetKinematics(), maxLinearVel};

    frc::DifferentialDriveVoltageConstraint m_voltage_constraint{IO.drivebase.GetFeedForward(), IO.drivebase.GetKinematics(), 12.5_V};

    //rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{theChonkster};

    frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc);

    std::vector<frc::Spline<5>::ControlVector> points;
    config.AddConstraint(m_elliptic_constraint);
    config.AddConstraint(m_voltage_constraint);
    config.AddConstraint(m_kinematic_constraint);

    config.SetReversed(true);

    io::CSVReader<6> csv("/home/lvuser/deploy/PathWeaver/Paths/BackTest");
    csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
    double x, y, dx, dy, ddx = 0, ddy = 0;
    while (csv.read_row(x, y, dx, dy, ddx, ddy))
    {
        //std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
        points.push_back({{x, dx, ddx}, {y, dy, ddy}});
    }

    m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(points, config);

    m_autoTimer.Reset();
    m_autoTimer.Start();

    IO.drivebase.ResetOdometry(m_trajectory.InitialPose());

    // for(auto x : m_trajectory.States())
    // {
    //     cout << x.pose.X().value() << "," << x.pose.Y().value() << endl;
    // }
}

// Execute the program
void BigBalleros::Run()
{
    switch (m_state)
    {
    case 0:
    {
        IO.drivebase.VisionAim(0, 40.0, 0.1);
    }
    case 10:
    {
        //IO.shooter.IntakeDeploy();

        auto reference = m_trajectory.Sample(m_autoTimer.Get());
        //cout << reference.pose.X().value() << "," << reference.pose.Y().value() << endl;

        auto speeds = IO.m_ramsete.Calculate(IO.drivebase.GetPose(), reference);

        IO.drivebase.MeasuredDrive(speeds.vx, speeds.omega);

        //cout << IO.drivebase.GetPose().X().value() << "," << IO.drivebase.GetPose().Y().value() << endl;

        if ((m_autoTimer.Get() > m_trajectory.TotalTime()))
        {
            NextState();
        }
        break;
    }
    case 4:
    {
        NextState();
        data = IO.RJV.Run(0.0);

        IO.shooter.SetVelocity(3000.0);
        IO.shooter.SetHoodAngle(56.0);

        if (data.filled)
        {

            if ((abs(3000.0 - IO.shooter.GetVelocity()) < 150.0) && (abs(IO.shooter.GetHoodAngle() - 56.0) < 1.5) && IO.drivebase.VisionAim(0.0, data.angle, 0.3))
            {
                IO.shooter.SetIntake(100.0);
                IO.shooter.SetIndexer(100.0);
                IO.shooter.SetFeeder(100.0);
                shotCounterOS = false;
            }
            else
            {
                if (!shotCounterOS && !((abs(3000.0 - IO.shooter.GetVelocity()) < 150.0) && (abs(IO.shooter.GetHoodAngle() - 56.0) < 1.5)))
                {
                    ++shotCounter;
                    shotCounterOS = true;
                }
                IO.shooter.SetFeeder(0.0);
            }
        }
        if (shotCounter >= 4)
            NextState();

        if (m_autoTimer.Get().value() > 5.0)
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

void BigBalleros::UpdateSmartDash()
{
    SmartDashboard::PutNumber("Auto State", m_state);
}