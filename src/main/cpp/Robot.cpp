// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/SlewRateLimiter.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/controller/RamseteController.h>
#include <frc/Preferences.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/Timer.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>
#include <wpi/json.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/GlobalDevices.h"
#include <UDPLogger.hpp>

#include <lib/DiffyDriveTrajectoryConstraint.hpp>
#include <DrivetrainModel.hpp>

#include <memory>
#include <thread>

void logToUDPLogger(UDPLogger &logger, std::vector<std::shared_ptr<rj::Loggable>> &loggables)
{
  auto target =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
  logger.InitLogger();
  while (true)
  {
    logger.CheckForNewClient();

    for (auto &loggable : loggables)
    {
      loggable->Log(logger);
    }

    logger.FlushLogBuffer();
    std::this_thread::sleep_until(target);
    target = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
  }
}

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override
  {
    // Flush NetworkTables every loop. This ensures that robot pose and other
    // values are sent during every iteration.
    SetNetworkTablesFlushEnabled(true);

    m_chooser.SetDefaultOption("None", "None");
    m_chooser.AddOption("TEST", "TEST");
    m_chooser.AddOption("A-Red", "A-Red");
    m_chooser.AddOption("A-Blue", "A-Blue");
    m_chooser.AddOption("B-Red", "B-Red");
    m_chooser.AddOption("B-Blue", "B-Blue");
    m_chooser.AddOption("Barrel", "Barrel");
    m_chooser.AddOption("Slalom", "Slalom");
    m_chooser.AddOption("Bounce", "Bounce");
    m_chooser.AddOption("Accuracy", "Accuracy");
    m_chooser.AddOption("PowerPort", "PowerPort");
    frc::SmartDashboard::PutData(&m_chooser);

    // Robot Preferences
    prefs = frc::Preferences::GetInstance();

    frc::SmartDashboard::PutNumber("VoltageL", 0.0);
    frc::SmartDashboard::PutNumber("VoltageR", 0.0);
    frc::SmartDashboard::PutNumber("Left Rate", 0.0);
    frc::SmartDashboard::PutNumber("Right Rate", 0.0);
    frc::SmartDashboard::PutString("Reference", "");
    frc::SmartDashboard::PutString("Pose", "");

    auto time_point = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(time_point);
    m_udp_logger.SetTitle(std::ctime(&time));
    m_logging_thread =
        std::thread(logToUDPLogger, std::ref(m_udp_logger), std::ref(loggables));
    m_logging_thread.detach();
  }

  void RobotPeriodic() override
  {
    m_drive.Periodic();

    frc::SmartDashboard::PutNumber("m_autoState", m_autoState);
    frc::SmartDashboard::PutNumber("m_autoTimer", m_autoTimer.Get().value());

    wpi::json j;
    frc::to_json(j, m_drive.GetPose());
    auto pose_json = j.dump(0);

    frc::SmartDashboard::PutString("Pose", pose_json);
  }

  void AutonomousInit() override
  {
    // Generate / load paths
    auto path = m_chooser.GetSelected();
    if (m_chooser.GetSelected() == "None")
      return;

    if (path == "A-Red")
    {
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d(40_in, 120_in, -35_deg), {frc::Translation2d(150_in, 60_in), frc::Translation2d(180_in, 150_in)},
          frc::Pose2d(360_in, 170_in, 0_deg),
          frc::TrajectoryConfig(10_fps, 10_fps_sq));
    }
    else if (path == "A-Blue")
    {
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d(45_in, 35_in, 0_deg), {frc::Translation2d(170_in, 35_in), frc::Translation2d(210_in, 110_in)},
          frc::Pose2d(360_in, 70_in, 0_deg),
          frc::TrajectoryConfig(10_fps, 10_fps_sq));
    }
    else if (path == "B-Red")
    {
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d(45_in, 150_in, -45_deg), {frc::Translation2d(150_in, 70_in), frc::Translation2d(210_in, 110_in)},
          frc::Pose2d(360_in, 160_in, 0_deg),
          frc::TrajectoryConfig(10_fps, 10_fps_sq));
    }
    else if (path == "B-Blue")
    {
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d(45_in, 60_in, 0_deg), {frc::Translation2d(170_in, 60_in), frc::Translation2d(240_in, 110_in)},
          frc::Pose2d(360_in, 20_in, -35_deg),
          frc::TrajectoryConfig(10_fps, 10_fps_sq));
    }
    else if (path == "Barrel")
    {
      wpi::SmallString<64> deployDirectory;
      frc::filesystem::GetDeployDirectory(deployDirectory);
      wpi::sys::path::append(deployDirectory, "output");
      wpi::sys::path::append(deployDirectory, "Barrel2Cop.wpilib.json");

      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
      // rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper};

      // frc::TrajectoryConfig config(3_fps, 3_fps_sq);
      // config.AddConstraint(m_drivetrain_constraint);

      // std::vector<frc::Spline<5>::ControlVector> points{
      //     {{0.8200924998284499, 3.048, 0.0},
      //      {-2.2497640842654225, 0.0, 0.0}},
      //     {{4.199915185617237, 0.900906462215082, 0.0},
      //      {-2.4408261854113773, -1.1291725521124159, 0.0}},
      //     {{4.595216084539903, 0.02595854361932215, 0.0},
      //      {-3.0864843203183976, -0.38072530641672664, 0.0}},
      //     {{3.7914375900638166, -0.7141589071175916, 0.0},
      //      {-3.718965758594662, 0.009115386025385419, 0.0}},
      //     {{3.060130927056886, 0.017305695746214766, 0.0},
      //      {-3.0074241405338644, 0.423989545782264, 0.0}},
      //     {{4.470037466547726, 2.1372534246575343, 0.0},
      //      {-2.177292252796267, -0.03461139149242953, 0.0}},
      //     {{6.986786523022028, -0.18288307284347294, 0.0},
      //      {-1.3735137583201815, 1.6403069464171405, 0.0}},
      //     {{6.143477938653675, -0.5116216604171, 0.0},
      //      {-0.7015022301516503, -0.059207625021598556, 0.0}},
      //     {{5.445113017223633, 0.13844556596971902, 0.0},
      //      {-1.4262205448432033, -1.3498442682047587, 0.0}},
      //     {{7.349145680367804, 1.0816059841384291, 0.0},
      //      {-3.494961915871818, -0.4412952415284783, 0.0}},
      //     {{8.699757085020243, -0.10383417447729038, 0.0},
      //      {-2.9415406573800875, 1.7219167267483777, 0.0}},
      //     {{7.685151444452069, -0.6390697865916417, 0.0},
      //      {-2.0916437246963566, 0.046118438207644274, 0.0}},
      //     {{6.249004325883201, -2.194032803204273, 0.0},
      //      {-2.138901586157174, 0.09337629966846173, 0.0}},
      //     {{0.7476206683592946, -0.6576164383561647, 0.0},
      //      {-2.2695291292115556, 0.0778756308579669, 0.0}}};
      // m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      //     points,
      //     config);
    }
    else if (path == "Slalom")
    {
      wpi::SmallString<64> deployDirectory;
      frc::filesystem::GetDeployDirectory(deployDirectory);
      wpi::sys::path::append(deployDirectory, "output");
      wpi::sys::path::append(deployDirectory, "Slalom.wpilib.json");

      m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);

      // rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper};

      // frc::TrajectoryConfig config(3_fps, 3_fps_sq);
      // config.AddConstraint(m_drivetrain_constraint);

      // std::vector<frc::Spline<5>::ControlVector> points{
      //     {{1.2351584436972485,
      //       1.7063822136828386,
      //       0.0},
      //      {-3.7716725451176836,
      //       0.03294174157688934,
      //       0.0}},
      //     {{2.447414533726755,
      //       1.0277823371989299,
      //       0.0},
      //      {-3.0733076236876413,
      //       2.0357996294517253,
      //       0.0}},
      //     {{4.535740447007931,
      //       2.5525901225666914,
      //       0.0},
      //      {-1.7668291276135544,
      //       -0.2249740447007933,
      //       0.0}},
      //     {{6.024887668976874,
      //       0.6935140520363634,
      //       0.0},
      //      {-2.2168223426885336,
      //       -0.5123579343646885,
      //       0.0}},
      //     {{6.7508695025234315,
      //       0.4845594808940161,
      //       0.0},
      //      {-3.0907148521989902,
      //       -0.5883936553713056,
      //       0.0}},
      //     {{7.7805583994232155,
      //       0.9950775054073535,
      //       0.0},
      //      {-3.878124008651766,
      //       0.0778756308579669,
      //       0.0}},
      //     {{8.70641312184571,
      //       0.10383417447729038,
      //       0.0},
      //      {-3.2118547224224945,
      //       1.271968637346792,
      //       0.0}},
      //     {{7.7805583994232155,
      //       -1.7478752703676994,
      //       0.0},
      //      {-2.156207281903389,
      //       -0.008652847873107383,
      //       0.0}},
      //     {{6.768175198269647,
      //       -0.3893781542898349,
      //       0.0},
      //      {-3.168590483056957,
      //       -0.951813266041817,
      //       0.0}},
      //     {{4.6222689257390055,
      //       -2.1978233597692864,
      //       0.0},
      //      {-4.016569574621485,
      //       0.0778756308579669,
      //       0.0}},
      //     {{2.4590569574621486,
      //       -1.133523071377073,
      //       0.0},
      //      {-3.0993677000720976,
      //       1.557512617159337,
      //       0.0}},
      //     {{1.4130438482124479,
      //       -0.6316578947368421,
      //       0.0},
      //      {-2.243175735950045,
      //       -0.060569935111751905,
      //       0.0}}};
      // m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      //     points,
      //     config);
    }
    else if (path == "Bounce")
    {
      wpi::SmallString<64> deployDirectory;
      frc::filesystem::GetDeployDirectory(deployDirectory);
      wpi::sys::path::append(deployDirectory, "output");
      wpi::sys::path::append(deployDirectory, "Bounce-#.wpilib.json");
      auto i = deployDirectory.rfind('#');

      // Load each path
      deployDirectory[i] = '1';
      auto t1 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
      auto s1 = t1.States();

      deployDirectory[i] = '2';
      auto t2 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
      auto s2 = t2.States();

      deployDirectory[i] = '3';
      auto t3 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
      auto s3 = t3.States();

      deployDirectory[i] = '4';
      auto t4 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
      auto s4 = t4.States();

      // Modify the time for each point in the second path
      // {
      //   auto start_time = s1.back().t + 10_ms;
      //   for (size_t i = 0; i < s2.size(); i++)
      //   {
      //     s2[i].t += start_time;
      //   }

      //   // Add the second path to the overall path
      //   s1.insert(s1.end(), s2.begin(), s2.end());
      // }

      // {
      //   auto start_time = s1.back().t + 10_ms;
      //   for (size_t i = 0; i < s3.size(); i++)
      //   {
      //     s3[i].t += start_time;
      //   }

      //   // Add the second path to the overall path
      //   s1.insert(s1.end(), s3.begin(), s3.end());
      // }

      // {
      //   auto start_time = s1.back().t + 10_ms;
      //   for (size_t i = 0; i < s4.size(); i++)
      //   {
      //     s4[i].t += start_time;
      //   }

      //   // Add the second path to the overall path
      //   s1.insert(s1.end(), s4.begin(), s4.end());
      // }

      {
        auto start_time = s1.back().t;
        for (size_t i = 0; i < s2.size(); i++)
        {
          s2[i].t += start_time;
        }

        // Add the second path to the overall path
        s1.insert(s1.end(), s2.begin(), s2.end());
      }

      {
        auto start_time = s1.back().t;
        for (size_t i = 0; i < s3.size(); i++)
        {
          s3[i].t += start_time;
        }

        // Add the second path to the overall path
        s1.insert(s1.end(), s3.begin(), s3.end());
      }

      {
        auto start_time = s1.back().t;
        for (size_t i = 0; i < s4.size(); i++)
        {
          s4[i].t += start_time;
        }

        // Add the second path to the overall path
        s1.insert(s1.end(), s4.begin(), s4.end());
      }

      // Save this Trajectory
      m_trajectory = frc::Trajectory(s1);
      
    }
    else if (path == "Accuracy")
    {
      frc::Pose2d green{60_in, 90_in, 0_deg};
      frc::Pose2d yellow{120_in, 90_in, 0_deg};
      frc::Pose2d blue{180_in, 90_in, 0_deg};
      frc::Pose2d red{240_in, 90_in, 0_deg};
      frc::Pose2d load{300_in, 90_in, 0_deg};
      frc::TrajectoryConfig config(Drivetrain::kMaxSpeed * 0.8, 5_fps_sq);

      // Green -> Load
      config.SetReversed(false);
      m_trajectory_IA[0] = frc::TrajectoryGenerator::GenerateTrajectory(
          green,
          {},
          load,
          config);

      // Load -> Yellow 1
      config.SetReversed(!config.IsReversed());
      m_trajectory_IA[1] = frc::TrajectoryGenerator::GenerateTrajectory(
          load,
          {},
          yellow,
          config);

      // Yellow -> Load
      config.SetReversed(!config.IsReversed());
      m_trajectory_IA[2] = frc::TrajectoryGenerator::GenerateTrajectory(
          yellow,
          {},
          load,
          config);

      // Load -> Yellow 2
      config.SetReversed(!config.IsReversed());
      m_trajectory_IA[3] = frc::TrajectoryGenerator::GenerateTrajectory(
          load,
          {},
          yellow,
          config);

      // Yellow -> Load
      config.SetReversed(!config.IsReversed());
      m_trajectory_IA[4] = frc::TrajectoryGenerator::GenerateTrajectory(
          yellow,
          {},
          load,
          config);

      // Load -> Blue
      config.SetReversed(!config.IsReversed());
      m_trajectory_IA[5] = frc::TrajectoryGenerator::GenerateTrajectory(
          load,
          {},
          blue,
          config);

      // blue -> Load
      config.SetReversed(!config.IsReversed());
      m_trajectory_IA[6] = frc::TrajectoryGenerator::GenerateTrajectory(
          blue,
          {},
          load,
          config);

      // Load -> Red
      config.SetReversed(!config.IsReversed());
      m_trajectory_IA[7] = frc::TrajectoryGenerator::GenerateTrajectory(
          load,
          {},
          red,
          config);

      // Red -> Load
      config.SetReversed(!config.IsReversed());
      m_trajectory_IA[8] = frc::TrajectoryGenerator::GenerateTrajectory(
          red,
          {},
          load,
          config);

      // Load -> Green
      config.SetReversed(!config.IsReversed());
      m_trajectory_IA[9] = frc::TrajectoryGenerator::GenerateTrajectory(
          load,
          {},
          green,
          config);

      // Cheat to load initial pose correclty
      m_trajectory = m_trajectory_IA[0];
    }
    else if (path == "PowerPort")
    {
      frc::Pose2d shoot{0_in, 0_in, 0_deg};
      frc::Pose2d load{-120_in, 0_in, 0_deg};        //300
      frc::TrajectoryConfig config(3_fps, 3_fps_sq); //10

      // Go to Load
      m_trajectory_PP = frc::TrajectoryGenerator::GenerateTrajectory(
          load,
          {},
          shoot,
          config);

      // Go To Shoot
      config.SetReversed(true);
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          shoot,
          {},
          load,
          config);
    }
    else if (path == "TEST")
    {
      // Get path from preferences for testing & tuning
      auto x0 = units::inch_t(prefs->GetDouble("x0", 0));
      auto y0 = units::inch_t(prefs->GetDouble("y0", 0));
      auto r0 = units::degree_t(prefs->GetDouble("r0", 0));

      auto x1 = units::inch_t(prefs->GetDouble("x1", 0));
      auto y1 = units::inch_t(prefs->GetDouble("y1", 0));

      auto x2 = units::inch_t(prefs->GetDouble("x2", 0));
      auto y2 = units::inch_t(prefs->GetDouble("y2", 0));

      auto x3 = units::inch_t(prefs->GetDouble("x3", 0));
      auto y3 = units::inch_t(prefs->GetDouble("y3", 0));
      auto r3 = units::degree_t(prefs->GetDouble("r3", 0));

      auto vel = units::meters_per_second_t(prefs->GetDouble("v", 2));
      auto accel = units::meters_per_second_squared_t(prefs->GetDouble("a", 2));

      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d(x0, y0, r0), {frc::Translation2d(x1, y1), frc::Translation2d(x2, y2)},
          frc::Pose2d(x3, y3, r3),
          frc::TrajectoryConfig(vel, accel));
    }

    // Reset State Vars
    m_autoTimer.Reset();
    m_autoTimer.Start();
    m_drive.ResetOdometry(m_trajectory.InitialPose());
    m_autoState = 0;
  }

  void AutonomousPeriodic() override
  {
    auto path = m_chooser.GetSelected();
    if (m_chooser.GetSelected() == "None")
    {
      return;
    }
    else if (path == "PowerPort")
    {
      AutoPowerPort();
    }
    else if (path == "Accuracy")
    {
      AutoAccuracy();
    }
    else
    {
      //
      // Detect when we cross the finish line and stop
      //

      // Distance from center of robot to the outside perimeter of the bumper
      auto BumperDist = 14_in;

      if ((path == "A-Red" || path == "A-Blue" || path == "B-Red" || path == "A-Blue"))
      {
        if (m_drive.GetPose().X() > (330.0_in - BumperDist))
        {
          m_drive.Drive(0_mps, 0_deg_per_s);
          m_autoTimer.Stop();
          return;
        }
      }
      else if (path == "Barrel" || path == "Slalom")
      {

        // if ((m_autoTimer.Get() > m_trajectory.TotalTime() / 2.0) && m_drive.GetPose().X() < (60.0_in + BumperDist))
        // {
        //   m_drive.Drive(0_mps, 0_deg_per_s);
        //   m_autoTimer.Stop();
        //   return;
        // }
        if(m_autoTimer.Get() > m_trajectory.TotalTime())
        {
          m_autoTimer.Reset();
          m_drive.Drive(0_mps, 0_deg_per_s);
          //m_autoTimer.Start();
        }
      }
      else if (path == "Bounce")
      {
        if (m_drive.GetPose().X() > (330.0_in - BumperDist))
        {
          m_drive.Drive(0_mps, 0_deg_per_s);
          m_autoTimer.Stop();
          return;
        }
      }

      //
      // Just Follow a Path...
      //
      auto reference = m_trajectory.Sample(m_autoTimer.Get());
      auto speeds = m_ramsete.Calculate(m_drive.GetPose(), reference);
      m_drive.Drive(speeds.vx, speeds.omega);

      wpi::json j;
      frc::to_json(j, reference);
      auto reference_json = j.dump(0);

      frc::SmartDashboard::PutString("Reference", reference_json);
    }
  }

  void TeleopPeriodic() override
  {
    // auto xSpeed = -m_speedLimiter.Calculate(
    //                   deadband(m_controller.GetRawAxis(1))) *
    //               Drivetrain::kMaxSpeed;

    // auto rot = -m_rotLimiter.Calculate(
    //                deadband(m_controller.GetRawAxis(2))) *
    //            Drivetrain::kMaxAngularSpeed; //3

    // m_drive.Drive(xSpeed, -rot);
    m_drive.Arcade(m_controller.GetRawAxis(1), (0.5 * m_controller.GetRawAxis(2)));
  }

  void DisabledPeriodic() override
  {
    m_drive.Drive(0_mps, 0_deg_per_s);
  }

  double deadband(double in)
  {
    if (fabs(in) < 0.05)
      return 0.0;

    return in;
  }

  void SimulationPeriodic() override { m_drive.SimulationPeriodic(); }

  void AutoPowerPort()
  {

    auto delay = units::second_t(prefs->GetDouble("Load_Delay", 3.0));

    if (m_autoState == 0)
    {
      // Go To reload
      auto elapsed = m_autoTimer.Get();
      auto reference = m_trajectory.Sample(elapsed);

      wpi::json j;
      frc::to_json(j, reference);
      auto reference_json = j.dump(0);

      frc::SmartDashboard::PutString("Reference", reference_json);

      auto speeds = m_ramsete.Calculate(m_drive.GetPose(), reference);
      m_drive.Drive(speeds.vx, speeds.omega);

      // Next State
      if (elapsed > (m_trajectory.TotalTime() + delay))
      {
        m_autoState = 1;
        m_autoTimer.Reset();
        m_autoTimer.Start();
      }
    }
    else if (m_autoState == 1)
    {
      // Go To Shoot
      auto elapsed = m_autoTimer.Get();
      auto reference = m_trajectory_PP.Sample(elapsed);
      auto speeds = m_ramsete.Calculate(m_drive.GetPose(), reference);
      m_drive.Drive(speeds.vx, speeds.omega);

      // Next State
      if (elapsed > m_trajectory_PP.TotalTime() + delay)
      {
        m_autoState = 0;
        m_autoTimer.Reset();
        m_autoTimer.Start();
      }
    }
    else
    {
      m_drive.Drive(0_mps, 0_deg_per_s);
    }
  }

  void AutoAccuracy()
  {
    // Load Delay
    auto delay = units::second_t(prefs->GetDouble("Load_Delay", 3.0));

    // Pathing
    auto elapsed = m_autoTimer.Get();
    auto reference = m_trajectory_IA[m_autoState].Sample(elapsed);
    auto speeds = m_ramsete.Calculate(m_drive.GetPose(), reference);
    m_drive.Drive(speeds.vx, speeds.omega);

    // Next State
    if (elapsed > (m_trajectory_IA[m_autoState].TotalTime() + delay))
    {
      m_autoState++;
      m_autoState %= 10;
      m_autoTimer.Reset();
      m_autoTimer.Start();
    }
  }

private:
  // TODO(Dereck): Change to PS4 controller
  frc::XboxController m_controller{0};

  UDPLogger m_udp_logger;
  std::thread m_logging_thread;

  std::vector<std::shared_ptr<rj::Loggable>> loggables{
      std::shared_ptr<rj::Loggable>(&m_globals),
      std::shared_ptr<rj::Loggable>(&m_drive),
  };

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  frc::SlewRateLimiter<units::scalar> m_speedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  Drivetrain m_drive{IsSimulation()};
  GlobalDevices m_globals;

  frc::Trajectory m_trajectory;
  frc::Trajectory m_trajectory_PP;
  frc::Trajectory m_trajectory_IA[10];
  frc::RamseteController m_ramsete;
  frc2::Timer m_autoTimer;

  frc::Preferences *prefs;

  frc::SendableChooser<std::string> m_chooser;

  // Auto State
  uint m_autoState = 0;
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
