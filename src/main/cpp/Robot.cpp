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
#include <frc/Timer.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>
#include <wpi/json.h>
#include <frc/DigitalInput.h>
#include <frc/Servo.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/GlobalDevices.h"
#include <UDPLogger.hpp>

#include <lib/DiffyDriveTrajectoryConstraint.hpp>
#include <DrivetrainModel.hpp>
#include <lib/EllipticAccelerationConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <lib/csv.h>

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

frc::Trajectory LoadWaypointCSV(std::string path, frc::TrajectoryConfig &config)
{
  std::vector<frc::Spline<5>::ControlVector> points;

  io::CSVReader<6> csv(path);
  csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
  double x, y, dx, dy, ddx = 0, ddy = 0;
  while (csv.read_row(x, y, dx, dy, ddx, ddy))
  {
    //std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
    points.push_back({{x, dx, ddx}, {y, dy, ddy}});
  }

  return frc::TrajectoryGenerator::GenerateTrajectory(
      points,
      config);
}

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override
  {
    // Flush NetworkTables every loop. This ensures that robot pose and other
    // values are sent during every iteration.
    //SetNetworkTablesFlushEnabled(true);

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

    SupplyCurrentLimitConfiguration config{true, 30.0, 40.0, 0.0};
    intake.ConfigSupplyCurrentLimit(config);

    auto maxLinearVel = 17.5_fps;
    auto maxCentripetalAcc = 1_SG;
    auto maxLinearAcc = 20_fps_sq;

    frc::EllipticAccelerationConstraint m_elliptic_constraint{maxCentripetalAcc, maxLinearAcc};

    frc::DifferentialDriveKinematicsConstraint m_kinematic_constraint{m_drive.GetKinematics(), maxLinearVel};

    frc::DifferentialDriveVoltageConstraint m_voltage_constraint{m_drive.GetFeedForward(), m_drive.GetKinematics(), 12.5_V};

    rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper, 0.065_psi}; //.125

    frc::TrajectoryConfig tjconfig(maxLinearVel, maxLinearAcc); //17.5 15
    tjconfig.AddConstraint(m_elliptic_constraint);
    tjconfig.AddConstraint(m_voltage_constraint);
    tjconfig.AddConstraint(m_kinematic_constraint);
    tjconfig.SetEndVelocity(maxLinearVel);

    m_trajectory = LoadWaypointCSV("/home/lvuser/deploy/PathWeaver/Paths/Slalom", tjconfig);
  }

  void RobotPeriodic() override
  {
    m_drive.Periodic();

    frc::SmartDashboard::PutNumber("m_autoState", m_autoState);
    frc::SmartDashboard::PutNumber("m_autoTimer", m_autoTimer.Get().value());

    frc::SmartDashboard::PutBoolean("SLOS", slewOS);

    wpi::json j;
    frc::to_json(j, m_drive.GetPose());
    pose_json = j.dump(0);

    frc::SmartDashboard::PutString("Pose", pose_json);

    double vel = m_drive.GetVel();
    double acc = (vel - prevVel) / 0.02;
    double smoothAcc = 0.0;

    vels.push_back(acc);
    if (vels.size() > 5)
      vels.erase(vels.begin());

    if (vels.size() == 5)
      smoothAcc = (vels[0] + vels[1] + vels[2] + vels[3] + vels[4]) / 5.0;

    prevVel = vel;
    topVel = (abs(vel) > abs(topVel)) ? vel : topVel;
    topAcc = (smoothAcc > topAcc) ? smoothAcc : topAcc;

    frc::SmartDashboard::PutNumber("topVel", topVel);
    frc::SmartDashboard::PutNumber("topAcc", topAcc);
  }

  void AutonomousInit() override
  {
    // Generate / load paths
    auto path = m_chooser.GetSelected();
    if (m_chooser.GetSelected() == "None")
      return;

    if (path == "A-Red")
    {
      // m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      //     frc::Pose2d(40_in, 120_in, -35_deg), {frc::Translation2d(150_in, 60_in), frc::Translation2d(180_in, 150_in)},
      //     frc::Pose2d(360_in, 170_in, 0_deg),
      //     frc::TrajectoryConfig(10_fps, 10_fps_sq));
      rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper, 0.12_psi};

      frc::TrajectoryConfig config(17_fps, 20_fps_sq); //17.5 15
      config.AddConstraint(m_drivetrain_constraint);

      std::vector<frc::Spline<5>::ControlVector> points{
          {{1.1033914773896933, 0.5995396966993753, 0.0},
           {-2.2827058258423114, 0.0, 0.0}},
          {{2.262940780896178, 0.49787070755549695, 0.0},
           {-2.421061140465244, -0.31594358296760655, 0.0}},
          {{2.737301859603376, 0.383797537112627, 0.0},
           {-2.9151872641185754, -0.31212688083231743, 0.0}},
          {{3.4356667810334183, 0.7971901461607085, 0.0},
           {-3.330253207987374, 0.026353393261511204, 0.0}},
          {{4.107678309201949, 0.4545960337610646, 0.0},
           {-2.921775612433953, 0.9750755506759081, 0.0}},
          {{5.348637343031634, 0.9684872023605307, 0.0},
           {-1.0770380841281824, 1.0014289439374187, 0.0}},
          {{6.235714815068963, 0.9289571124682645, 0.0},
           {-0.43796829753654026, 0.013176696630755602, 0.0}},
          {{7.513854388252247, 0.8498969326837296, 0.0},
           {-0.3127896795443629, 0.019765044946133237, 0.0}},
          {{11.943525972689221, 0.6654231798531534, 0.0},
           {-0.20737610649831886, 0.0, 0.0}}};
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          points,
          config);

      intakeRetention.SetAngle(100);
    }
    else if (path == "A-Blue")
    {
      rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper, 0.04_psi};

      frc::TrajectoryConfig config(17.5_fps, 17.5_fps_sq); //17.5 15
      config.AddConstraint(m_drivetrain_constraint);

      std::vector<frc::Spline<5>::ControlVector> points{
          {{1.0, 3.048, 0.0},
           {-2.25, 0.0, 0.0}},
          {{4.094501612571194, 1.5350851574830164, 0.0},
           {-3.8177909833253283, 0.5336562135455978, 0.0}},
          {{4.621569477801414, 0.24376888766897675, 0.0},
           {-2.6252999382419544, 1.0936658203527072, 0.0}},
          {{5.174990736293145, 1.1859026967679958, 0.0},
           {-1.7161078707198245, 0.29647567419199894, 0.0}},
          {{6.8681962533452285, 1.1679189893505757, 0.0},
           {-2.0916437246963566, -0.21746497594941375, 0.0}},
          {{8.67999204007411, 1.0409590338296848, 0.0},
           {-2.361766005626844, 0.0, 0.0}}};
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          points,
          config);

      intakeRetention.SetAngle(100);
    }
    else if (path == "B-Red")
    {
      rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper, 0.06_psi};

      frc::TrajectoryConfig config(17_fps, 17.5_fps_sq); //17.5 15
      config.AddConstraint(m_drivetrain_constraint);

      std::vector<frc::Spline<5>::ControlVector> points{
          {{1.1231565223358266, 1.0, 0.0},
           {-0.7344439717285393, 0.0, 0.0}},
          {{2.3222359157345775, 0.45459603376106505, 0.0},
           {-1.5250457695738697, -1.1266075619295959, 0.0}},
          {{3.8704977698483503, 1.9501511013518154, 0.0},
           {-3.0074241405338644, 0.00658834831537769, 0.0}},
          {{5.484643107115899, 1.6024497343472723, 0.0},
           {-1.6897544774583135, 0.9684807432829056, 0.0}},
          {{8.950114321004598, 0.9684872023605298, 0.0},
           {-0.6487954436286283, -0.013176696630755658, 0.0}}};
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          points,
          config);

      intakeRetention.SetAngle(100);
    }
    else if (path == "B-Blue")
    {
      rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper, 0.04_psi};

      frc::TrajectoryConfig config(17.5_fps, 17.5_fps_sq); //17.5 15
      config.AddConstraint(m_drivetrain_constraint);

      std::vector<frc::Spline<5>::ControlVector> points{
          {{1.0, 3.048, 0.0},
           {-2.25, 0.0, 0.0}},
          {{4.733571399162836, 1.6221801185664946, 0.0},
           {-2.8229503877032873, 0.7315713744804807, 0.0}},
          {{6.1764196802305635, 1.2913162698140406, 0.0},
           {-1.702931174089069, 0.026353393261511426, 0.0}},
          {{7.6917397927674465, 0.8862436201948019, 0.0},
           {-2.7834202978110207, -0.3942415949459662, 0.0}},
          {{8.719522129966377, 0.5007144719687098, 0.0},
           {-3.10624936526453, 0.01317669663075538, 0.0}}};
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          points,
          config);

      intakeRetention.SetAngle(100);
    }
    else if (path == "Barrel")
    {

      // wpi::SmallString<64> deployDirectory;
      // frc::filesystem::GetDeployDirectory(deployDirectory);
      // wpi::sys::path::append(deployDirectory, "output");
      // wpi::sys::path::append(deployDirectory, "Barrel2Cop.wpilib.json");

      // m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);

      auto maxLinearVel = 17.5_fps;
      auto maxCentripetalAcc = 1.1_SG;
      auto initCentripetalAcc = 0.875_SG;
      auto maxLinearAcc = 15_fps_sq;
      auto initLinearAcc = 10_fps_sq;

      frc::EllipticAccelerationConstraint m_elliptic_constraint{maxCentripetalAcc, maxLinearAcc};

      frc::DifferentialDriveKinematicsConstraint m_kinematic_constraint{m_drive.GetKinematics(), maxLinearVel};

      frc::DifferentialDriveVoltageConstraint m_voltage_constraint{m_drive.GetFeedForward(), m_drive.GetKinematics(), 12.5_V};

      rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper, 0.065_psi}; //.125

      frc::Translation2d m_blBoundingBox{120_in, -150_in};
      frc::Translation2d m_trBoundingBox{210_in, -90_in};
      frc::EllipticAccelerationConstraint m_initial_elliptic_constraint{initCentripetalAcc, initLinearAcc};
      frc::RectangularRegionConstraint<frc::EllipticAccelerationConstraint> m_firstTurnConstraint{m_blBoundingBox, m_trBoundingBox, m_initial_elliptic_constraint};

      frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc); //17.5 15
      config.AddConstraint(m_elliptic_constraint);
      config.AddConstraint(m_voltage_constraint);
      config.AddConstraint(m_kinematic_constraint);
      config.SetEndVelocity(maxLinearVel);
      config.AddConstraint(m_firstTurnConstraint);

      std::vector<frc::Spline<5>::ControlVector> points;

      io::CSVReader<6> csv("/home/lvuser/deploy/PathWeaver/Paths/Barrel2Cop.path");
      csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
      double x, y, dx, dy, ddx = 0, ddy = 0;
      while (csv.read_row(x, y, dx, dy, ddx, ddy))
      {
        //std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
        points.push_back({{x, dx, ddx}, {y, dy, ddy}});
      }

      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          points,
          config);

      /*std::vector<frc::Spline<5>::ControlVector> points{
          {{1.2483351403280039, 1.6866171687367046, 0.0},
           {-2.2695291292115556, 0.01317669663075538, 0.0}},
          {{3.8704977698483503, 1.357199752967818, 0.0},
           {-2.1509388595347563, 0.026353393261511204, 0.0}},
          {{4.7994548823166125, -0.019765044946132626, 0.0},
           {-3.02060083716462, -1.0541357304604406, 0.0}},
          {{3.784849241748439, -1.0673124270911964, 0.0},
           {-3.8902628147944833, 0.01976504494613307, 0.0}},
          {{2.829538736018665, 0.006588348315378578, 0.0},
           {-3.060130927056886, 1.3044929664447955, 0.0}},
          {{3.837556028271461, 0.9882522473066624, 0.0},
           {-2.157527207850134, 0.0, 0.0}},
          {{6.143477938653675, 1.1002541686680836, 0.0},
           {-2.1970572977424006, 0.20423879777671017, 0.0}},
          {{6.953844781445139, 0.01317669663075538, 0.0},
           {-1.4921040279969808, 0.8103668427914638, 0.0}},
          {{6.229126466753585, -1.1002541686680827, 0.0},
           {-0.8596225897207165, -0.00658834831537769, 0.0}},
          {{5.445113017223633, 0.08564852809990953, 0.0},
           {-1.834698140396624, -1.2913162698140397, 0.0}},
          {{6.281833253276608, 0.4545960337610646, 0.0},
           {-3.1589561517875526, -0.3294174157688876, 0.0}},
          {{7.6917397927674465, 1.666852123790573, 0.0},
           {-3.896851163109861, -0.01317669663075538, 0.0}},
          {{8.811759006381665, 0.013176696630754492, 0.0},
           {-3.060130927056886, 1.2781395731832847, 0.0}},
          {{7.56656117477527, -1.4757900226446186, 0.0},
           {-2.309059219103822, -0.01317669663075538, 0.0}},
          {{4.542509298016881, -1.5701995764296477, 0.0},
           {-2.3024708707884445, -0.01058716807032005, 0.0}},
          {{2.8558921292801758, -1.3220538892623614, 0.0},
           {-2.3222359157345775, -0.005654328072778406, 0.0}},
          {{0.576323612159473, -0.7576600562684417, 0.0},
           {-2.309059219103822, -0.00658834831537769, 0.0}}};
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          points,
          config);*/
    }
    else if (path == "Slalom")
    {
      // wpi::SmallString<64> deployDirectory;
      // frc::filesystem::GetDeployDirectory(deployDirectory);
      // wpi::sys::path::append(deployDirectory, "output");
      // wpi::sys::path::append(deployDirectory, "Slalom.wpilib.json");

      // m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);

      // rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper, 0.125_psi}; //0.04

      // frc::TrajectoryConfig config(17.5_fps, 15.5_fps_sq);
      // config.AddConstraint(m_drivetrain_constraint);

      auto maxLinearVel = 17.5_fps;
      auto maxCentripetalAcc = 1_SG;
      auto maxLinearAcc = 15_fps_sq;

      frc::EllipticAccelerationConstraint m_elliptic_constraint{maxCentripetalAcc, maxLinearAcc};

      frc::DifferentialDriveKinematicsConstraint m_kinematic_constraint{m_drive.GetKinematics(), maxLinearVel};

      frc::DifferentialDriveVoltageConstraint m_voltage_constraint{m_drive.GetFeedForward(), m_drive.GetKinematics(), 12.5_V};

      rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper, 0.065_psi}; //.125

      frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc); //17.5 15
      config.AddConstraint(m_elliptic_constraint);
      config.AddConstraint(m_voltage_constraint);
      config.AddConstraint(m_kinematic_constraint);
      config.SetEndVelocity(maxLinearVel);

      std::vector<frc::Spline<5>::ControlVector> points;

      io::CSVReader<6> csv("/home/lvuser/deploy/PathWeaver/Paths/Slalom");
      csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
      double x, y, dx, dy, ddx = 0, ddy = 0;
      while (csv.read_row(x, y, dx, dy, ddx, ddy))
      {
        //std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
        points.push_back({{x, dx, ddx}, {y, dy, ddy}});
      }

      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          points,
          config);

      //m_trajectory = LoadWaypointCSV("/home/lvuser/deploy/PathWeaver/Paths/Slalom", config);

      /*
      std::vector<frc::Spline<5>::ControlVector> points{
          {{1.1875, 1.7063822136828386, 0.0},
           {-3.8112026350099506, 0.03294174157688934, 0.0}},
          {{2.2168223426885336, 0.4677727303918209, 0.0},
           {-3.0667192753722645, 0.42824264049955474, 0.0}},
          {{3.0733076236876413, 0.6905119844767379, 0.0},
           {-2.394707747203733, 0.33672982243883576, 0.0}},
          {{4.325093803609414, 0.918827777267091, 0.0},
           {-2.1377621629040005, 0.07550246313831972, 0.0}},
          {{5.83382556783092, 0.7457073233529692, 0.0},
           {-2.2299990393192894, -0.28102917100637403, 0.0}},
          {{6.637604062307006, 0.4845594808940161, 0.0},
           {-2.710948466341866, -0.5883936553713056, 0.0}},
          {{7.0790233994373155, 0.5534212584917313, 0.0},
           {-3.574022095656351, -0.6983649214300418, 0.0}},
          {{8.021157208536335, 1.087077472037329, 0.0},
           {-4.048383174363549, 0.0, 0.0}},
          {{8.778817264804777, 0.046118438207644274, 0.0},
           {-3.303899814725863, 0.7115416180607976, 0.0}},
          {{8.330809579359089, -0.4591601937325714, 0.0},
           {-2.4935329719343993, 0.23346557414699634, 0.0}},
          {{7.375499073629315, -0.6851882247992878, 0.0},
           {-2.658241679818843, -0.7444833596376861, 0.0}},
          {{6.584897275783985, -1.304492966444795, 0.0},
           {-3.6794356687023955, -0.9421338090990186, 0.0}},
          {{4.582039387909147, -1.1741143844288655, 0.0},
           {-4.028618129417416, 0.009252789704710477, 0.0}},
          {{3.0667192753722645, -1.5284968091676392, 0.0},
           {-3.8112026350099506, 0.5336562135455982, 0.0}},
          {{0.8135041515130722, -0.6851882247992864, 0.0},
           {-1.08362643244356, 0.48094942702257615, 0.0}}};
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          points,
          config);
          */
    }
    else if (path == "Bounce")
    {
      // wpi::SmallString<64> deployDirectory;
      // frc::filesystem::GetDeployDirectory(deployDirectory);
      // wpi::sys::path::append(deployDirectory, "output");
      // wpi::sys::path::append(deployDirectory, "Bounce-#.wpilib.json");
      // auto i = deployDirectory.rfind('#');

      // // Load each path
      // deployDirectory[i] = '1';
      // auto t1 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
      // auto s1 = t1.States();

      // deployDirectory[i] = '2';
      // auto t2 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
      // auto s2 = t2.States();

      // deployDirectory[i] = '3';
      // auto t3 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
      // auto s3 = t3.States();

      // deployDirectory[i] = '4';
      // auto t4 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
      // auto s4 = t4.States();

      auto maxLinearVel = 17.5_fps;
      auto maxCentripetalAcc = 0.7_SG;
      auto maxLinearAcc = 13.5_fps_sq;

      frc::EllipticAccelerationConstraint m_elliptic_constraint{maxCentripetalAcc, maxLinearAcc};

      frc::DifferentialDriveKinematicsConstraint m_kinematic_constraint{m_drive.GetKinematics(), maxLinearVel};

      frc::DifferentialDriveVoltageConstraint m_voltage_constraint{m_drive.GetFeedForward(), m_drive.GetKinematics(), 12.5_V};

      rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper, 0.065_psi}; //.125

      frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc); //17.5 15
      config.AddConstraint(m_elliptic_constraint);
      config.AddConstraint(m_voltage_constraint);
      config.AddConstraint(m_kinematic_constraint);
      config.SetEndVelocity(maxLinearVel);

      std::vector<frc::Spline<5>::ControlVector> p1;
      {
        io::CSVReader<6> csv("/home/lvuser/deploy/PathWeaver/Paths/Bounce-1");
        csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
        double x, y, dx, dy, ddx = 0, ddy = 0;
        while (csv.read_row(x, y, dx, dy, ddx, ddy))
        {
          //std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
          p1.push_back({{x, dx, ddx}, {y, dy, ddy}});
        }
      }

      std::vector<frc::Spline<5>::ControlVector> p2;
      {
        io::CSVReader<6> csv("/home/lvuser/deploy/PathWeaver/Paths/Bounce-2");
        csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
        double x, y, dx, dy, ddx = 0, ddy = 0;
        while (csv.read_row(x, y, dx, dy, ddx, ddy))
        {
          //std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
          p2.push_back({{x, dx, ddx}, {y, dy, ddy}});
        }
      }

      std::vector<frc::Spline<5>::ControlVector> p3;
      {
        io::CSVReader<6> csv("/home/lvuser/deploy/PathWeaver/Paths/Bounce-3");
        csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
        double x, y, dx, dy, ddx = 0, ddy = 0;
        while (csv.read_row(x, y, dx, dy, ddx, ddy))
        {
          //std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
          p3.push_back({{x, dx, ddx}, {y, dy, ddy}});
        }
      }

      std::vector<frc::Spline<5>::ControlVector> p4;
      {
        io::CSVReader<6> csv("/home/lvuser/deploy/PathWeaver/Paths/Bounce-4");
        csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
        double x, y, dx, dy, ddx = 0, ddy = 0;
        while (csv.read_row(x, y, dx, dy, ddx, ddy))
        {
          //std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
          p4.push_back({{x, dx, ddx}, {y, dy, ddy}});
        }
      }
      auto t1 = frc::TrajectoryGenerator::GenerateTrajectory(
          p1,
          config);
      auto s1 = t1.States();

      config.SetReversed(true);

      auto t2 = frc::TrajectoryGenerator::GenerateTrajectory(
          p2,
          config);
      auto s2 = t2.States();

      config.SetReversed(false);

      auto t3 = frc::TrajectoryGenerator::GenerateTrajectory(
          p3,
          config);
      auto s3 = t3.States();

      config.SetReversed(true);

      auto t4 = frc::TrajectoryGenerator::GenerateTrajectory(
          p4,
          config);
      auto s4 = t4.States();

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

      intakeRetention.SetAngle(100.0);
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
        intake.Set(-1.0);
        if (m_drive.GetPose().X() > (350.0_in - BumperDist))
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
        if (m_autoTimer.Get() > m_trajectory.TotalTime())
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
      if (m_autoTimer.Get().value() > 7.4 && path == "Barrel")
        m_drive.SetImpel(0.0);
      else
        m_drive.SetImpel(-1.0);

      auto reference = m_trajectory.Sample(m_autoTimer.Get());
      auto speeds = m_ramsete.Calculate(m_drive.GetPose(), reference);
      m_drive.Drive(speeds.vx, speeds.omega);

      wpi::json j;
      frc::to_json(j, reference);
      auto reference_json = j.dump(0);

      frc::SmartDashboard::PutString("Reference", reference_json);

      std::cout << reference.pose.X().value() << "," << reference.pose.Y().value() << "," << reference.pose.Rotation().Degrees().value() /* << "," << m_drive.GetPose().X().value() << "," << m_drive.GetPose().Y().value() << "," << m_drive.GetPose().Rotation().Degrees().value()*/ << "," << speeds.vx.value() << "," << speeds.omega.value() << std::endl;
    }
  }

  void TeleopInit() override
  {
    SupplyCurrentLimitConfiguration config{true, 40.0, 50.0, 0.0};
    m_drive.impel.ConfigSupplyCurrentLimit(config);
    m_drive.impel2.ConfigSupplyCurrentLimit(config);
    intakeRetention.SetAngle(150.0);
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

    double forward = deadband(pow(m_controller.GetRawAxis(1), 1));
    // if ((fabs(forward) > 0.5) && !slewOS)
    // {
    //   Slew.Reset();
    //   Slew.Start();
    //   slewOS = true;
    //   std::cout << "AHHHHHHHHHHH" << std::endl;
    // }
    // if (Slew.Get() < 0.5)
    // {
    //   forward = m_speedLimiter.Calculate(forward);
    // }
    // else if (!impelOS && (Slew.Get() > 0.5) && slewOS)
    // {
    //   SupplyCurrentLimitConfiguration config{true, 25.0, 25.0, 0.0};
    //   m_drive.impel.ConfigSupplyCurrentLimit(config);
    //   m_drive.impel2.ConfigSupplyCurrentLimit(config);
    //   impelOS = true;
    // }
    // else
    // {
    //   double yeet = m_speedLimiter.Calculate(forward);
    // }

    m_drive.Arcade(forward, deadband(0.5 * pow(m_controller.GetRawAxis(2), 1)));
    double intakeSpd = ((m_controller.GetRawAxis(3) / 2.0) + 0.5) - ((m_controller.GetRawAxis(4) / 2.0) + 0.5);

    if (m_controller.GetBumper(frc::GenericHID::JoystickHand::kRightHand))
      m_drive.SetImpel(-1.0);
    else
      m_drive.SetImpel(0.0);

    if (m_controller.GetBumper(frc::GenericHID::JoystickHand::kLeftHand))
      intakeRetention.SetAngle(100.0);

    intake.Set(intakeSpd);
    // m_drive.SetImpel(-1.0);
  }

  void DisabledInit() override
  {
    brakeTimer.Reset();
    brakeTimer.Start();
  }

  void DisabledPeriodic() override
  {
    m_drive.Drive(0_mps, 0_deg_per_s);
    slewOS = false;
    impelOS = false;
    m_speedLimiter.Reset(0.0);
    if (brakeTimer.Get() > 2.0)
    {
    }
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

  frc::Timer Slew;
  bool slewOS = false;
  bool impelOS = false;

  std::vector<std::shared_ptr<rj::Loggable>> loggables{
      std::shared_ptr<rj::Loggable>(&m_globals),
      std::shared_ptr<rj::Loggable>(&m_drive),
  };

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  frc::SlewRateLimiter<units::scalar> m_speedLimiter{2 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{2 / 1_s};

  Drivetrain m_drive{IsSimulation()};
  GlobalDevices m_globals;

  frc::Trajectory m_trajectory;
  frc::Trajectory m_trajectory_PP;
  frc::Trajectory m_trajectory_IA[10];
  frc::RamseteController m_ramsete{2.0, 0.7};
  frc2::Timer m_autoTimer;

  frc::Preferences *prefs;

  frc::SendableChooser<std::string> m_chooser;

  // Auto State
  uint m_autoState = 0;

  double prevVel = 0.0;
  double topVel = 0.0;
  double topAcc = 0.0;
  double prevCol = 0.0;
  int colCt = 0;

  vector<double> vels;

  string pose_json;

  WPI_TalonFX intake{8};

  frc::Servo intakeRetention{9};

  frc::Timer brakeTimer;
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
