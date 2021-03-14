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
    pose_json = j.dump(0);

    frc::SmartDashboard::PutString("Pose", pose_json);

    double vel = m_drive.GetVel();
    double acc = (vel - prevVel) / 0.02;
    double smoothAcc = 0.0;
    if (colCt < 10)
    {
      prevCol += acc;
      colCt++;
    }

    if (colCt >= 10)
    {
      smoothAcc = prevCol / 10.0;
      colCt = 0;
      prevCol = 0.0;
    }
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
      rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper, 0.1_psi};

      frc::TrajectoryConfig config(16_fps, 13_fps_sq); //17.5 15
      config.AddConstraint(m_drivetrain_constraint);

      std::vector<frc::Spline<5>::ControlVector> points{
          {{1.0, 0.7688146572428465, 0.0},
           {-2.25, -0.019765044946133292, 0.0}},
          {{2.355177657311467, 0.48387816747576157, 0.0},
           {-2.355177657311467, -0.4171459884144158, 0.0}},
          {{2.737301859603376, 0.3387625578793974, 0.0},
           {-2.9151872641185754, -0.3381660542059067, 0.0}},
          {{3.40272503945653, 0.5007144719687098, 0.0},
           {-3.336841556302752, 0.13176696630755425, 0.0}},
          {{3.765084196802306, 0.3821242022919096, 0.0},
           {-2.9876590955877314, 0.61930474164551, 0.0}},
          {{3.989088039525149, 0.5019686743419465, 0.0},
           {-1.2549234886433818, 0.5814333535102015, 0.0}},
          {{4.4370957249708365, 0.6539948934743014, 0.0},
           {-0.8793876346668497, 0.41720066299364955, 0.0}},
          {{6.183008028545942, 0.7708367528991973, 0.0},
           {-0.6487954436286283, 0.013176696630755491, 0.0}},
          {{8.600931860289577, 0.6654231798531534, 0.0},
           {-0.6949138818362726, 0.026353393261511426, 0.0}}};
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          points,
          config);
    }
    else if (path == "A-Blue")
    {
      rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper, 0.1_psi};

      frc::TrajectoryConfig config(7_fps, 7_fps_sq); //17.5 15
      config.AddConstraint(m_drivetrain_constraint);

      std::vector<frc::Spline<5>::ControlVector> points{
          {{1.0, 3.048, 0.0},
           {-2.25, 0.0, 0.0}},
          {{3.692612365333151, 1.5350851574830164, 0.0},
           {-3.8441443765868386, 0.5336562135455978, 0.0}},
          {{4.45686076991697, 0.24376888766897675, 0.0},
           {-2.5396514101420435, 1.0936658203527072, 0.0}},
          {{5.1684023879777685, 1.2781395731832825, 0.0},
           {-1.5382224662046253, 0.6983649214300423, 0.0}},
          {{6.874784601660605, 1.1716499185339804, 0.0},
           {-2.2563524325808, -0.2697165791353041, 0.0}},
          {{8.67999204007411, 1.0409590338296848, 0.0},
           {-2.361766005626844, 0.0, 0.0}}};
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          points,
          config);
    }
    else if (path == "B-Red")
    {
      rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper, 0.1_psi};

      frc::TrajectoryConfig config(7_fps, 7_fps_sq); //17.5 15
      config.AddConstraint(m_drivetrain_constraint);

      std::vector<frc::Spline<5>::ControlVector> points{
          {{1.0, 1.0, 0.0},
           {-2.25, 0.0, 0.0}},
          {{2.2695291292115556, 1.6339103822136831, 0.0},
           {-1.4921040279969808, 0.006588348315377912, 0.0}},
          {{3.798025938379195, 1.6536754271598153, 0.0},
           {-3.033777533795375, 0.0, 0.0}},
          {{5.3792295340698555, 1.5711135984846305, 0.0},
           {-1.5448108145200028, 0.8350248254711028, 0.0}},
          {{8.851289096273932, 1.2968588485555488, 0.0},
           {-1.4657506347354698, -0.005542578741508208, 0.0}}};
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          points,
          config);
    }
    else if (path == "B-Blue")
    {
      rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper, 0.1_psi};

      frc::TrajectoryConfig config(7_fps, 7_fps_sq); //17.5 15
      config.AddConstraint(m_drivetrain_constraint);

      std::vector<frc::Spline<5>::ControlVector> points{
          {{1.0, 3.048, 0.0},
           {-2.25, 0.0, 0.0}},
          {{4.700629657585947, 1.5689516109535795, 0.0},
           {-3.07989597200302, 0.8588567277300433, 0.0}},
          {{6.143477938653675, 1.2913162698140406, 0.0},
           {-1.5316341178892476, 0.026353393261511426, 0.0}},
          {{7.639033006244425, 0.9189023186463852, 0.0},
           {-3.0271891854799975, -0.5459804055419246, 0.0}},
          {{8.67999204007411, 0.5007144719687098, 0.0},
           {-3.475196870925685, 0.01317669663075538, 0.0}}};
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          points,
          config);
    }
    else if (path == "Barrel")
    {
      // wpi::SmallString<64> deployDirectory;
      // frc::filesystem::GetDeployDirectory(deployDirectory);
      // wpi::sys::path::append(deployDirectory, "output");
      // wpi::sys::path::append(deployDirectory, "Barrel2Cop.wpilib.json");

      // m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);

      rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper, 0.1_psi};

      frc::TrajectoryConfig config(7_fps, 7_fps_sq); //17.5 15
      config.AddConstraint(m_drivetrain_constraint);

      std::vector<frc::Spline<5>::ControlVector> points{
          {{0.8200924998284499, 3.048, 0.0},
           {-2.2497640842654225, 0.0, 0.0}},
          {{3.9297929046867495, 0.4282426404995543, 0.0},
           {-2.394707747203733, -0.006588348315378134, 0.0}},
          {{4.423919028340081, 0.22362380950435873, 0.0},
           {-2.57918150003431, -0.22883838178909777, 0.0}},
          {{4.595216084539903, 0.02595854361932215, 0.0},
           {-3.0864843203183976, -0.38072530641672664, 0.0}},
          {{4.33827050024017, -0.2555665650299187, 0.0},
           {-3.514726960817951, -0.22607721769718894, 0.0}},
          {{3.7914375900638166, -0.7141589071175916, 0.0},
           {-3.718965758594662, 0.009115386025385419, 0.0}},
          {{3.3170765113566185, -0.2558336419542251, 0.0},
           {-3.5279036574487064, 0.22445657762015062, 0.0}},
          {{3.060130927056886, 0.017305695746214766, 0.0},
           {-3.0074241405338644, 0.423989545782264, 0.0}},
          {{3.303899814725863, 0.3605923778698733, 0.0},
           {-2.513298016880533, 0.41300018887636014, 0.0}},
          {{4.496390859809237, 0.434830988814932, 0.0},
           {-2.3815310505729768, 0.006588348315376802, 0.0}},
          {{6.894549646606739, -0.01317669663075538, 0.0},
           {-1.5513991628353807, 1.5087317642215057, 0.0}},
          {{6.545367185891719, -0.05270678652302152, 0.0},
           {-0.8332691964592056, 0.052706786523021965, 0.0}},
          {{6.051241062238384, -0.34918246071502157, 0.0},
           {-0.7542090166746724, -0.07906017978453295, 0.0}},
          {{5.478054758800521, -0.19765044946133248, 0.0},
           {-1.004566252659027, -0.25035723598435444, 0.0}},
          {{5.3726411857544765, 0.019765044946134402, 0.0},
           {-1.6370476909352913, -0.5995396966993758, 0.0}},
          {{7.309615590475538, 1.0816059841384291, 0.0},
           {-3.718965758594662, -0.4412952415284783, 0.0}},
          {{8.429634804089755, -0.013176696630754492, 0.0},
           {-3.3697832978796405, 0.7576600562684419, 0.0}},
          {{8.284691141151445, -0.24972271704421992, 0.0},
           {-2.7307135112879988, 0.3629367298451762, 0.0}},
          {{7.579737871406025, -0.5731863034378639, 0.0},
           {-2.361766005626844, 0.013176696630754936, 0.0}},
          {{6.367481781376519, -1.5086361674683444, 0.0},
           {-2.3156475674192003, 0.03398425362189381, 0.0}},
          {{3.0535425787415083, -1.8952508941697392, 0.0},
           {-2.3815310505729768, -0.03719701460318264, 0.0}},
          {{0.6817371852055171, -0.5204795169148426, 0.0},
           {-2.427649488780622, 0.013176696630756268, 0.0}}};
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          points,
          config);
    }
    else if (path == "Slalom")
    {
      // wpi::SmallString<64> deployDirectory;
      // frc::filesystem::GetDeployDirectory(deployDirectory);
      // wpi::sys::path::append(deployDirectory, "output");
      // wpi::sys::path::append(deployDirectory, "Slalom.wpilib.json");

      // m_trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);

      rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper, 0.1_psi};

      frc::TrajectoryConfig config(17.5_fps, 13_fps_sq);
      config.AddConstraint(m_drivetrain_constraint);

      std::vector<frc::Spline<5>::ControlVector> points{
          {{0.8135041515130722, 1.7063822136828386, 0.0},
           {-3.8112026350099506, 0.03294174157688934, 0.0}},
          {{2.2168223426885336, 0.4677727303918209, 0.0},
           {-3.0667192753722645, 0.42824264049955474, 0.0}},
          {{3.0733076236876413, 0.7787023463356992, 0.0},
           {-2.394707747203733, 0.3613962455806413, 0.0}},
          {{4.6545112193783025, 0.9816638989912851, 0.0},
           {-2.262940780896178, -0.01317669663075538, 0.0}},
          {{5.735000343100255, 0.6343626836884848, 0.0},
           {-2.4078844438344884, -0.24343475592221508, 0.0}},
          {{6.571720579153229, 0.4845594808940161, 0.0},
           {-2.954717354010843, -0.5883936553713056, 0.0}},
          {{6.914314691552873, 0.28698266567154695, 0.0},
           {-3.5476687023948394, -0.3646733696543395, 0.0}},
          {{7.441382556783093, 0.4054078102558327, 0.0},
           {-4.0417948260481715, -0.10027748387834926, 0.0}},
          {{8.126570781582378, 0.3219497603242634, 0.0},
           {-3.8639094215329717, 0.2700132398513276, 0.0}},
          {{8.423046455774378, -0.05929513483840054, 0.0},
           {-3.2511930282028407, 0.4480076854456865, 0.0}},
          {{7.968450422013312, -0.3924668113337224, 0.0},
           {-2.671418376449599, 0.15424064260709805, 0.0}},
          {{7.250320455637138, -0.5270678652302188, 0.0},
           {-2.7768319494956426, -0.46777273039182043, 0.0}},
          {{6.848431208399094, -0.1844737528305771, 0.0},
           {-3.376371646195018, -0.3491824607150211, 0.0}},
          {{6.611250669045496, -0.25159249244867143, 0.0},
           {-3.653082275440884, -0.24744924401458812, 0.0}},
          {{6.038064365607632, -0.622512088798959, 0.0},
           {-4.054971522678928, -0.2963626054758895, 0.0}},
          {{4.628157826116792, -1.0059734155124578, 0.0},
           {-4.173561792355726, 0.0028691170151164958, 0.0}},
          {{3.02060083716462, -0.8486536601014543, 0.0},
           {-4.008853084471283, 0.385997154367013, 0.0}},
          {{2.012583544911823, -0.42165429218417616, 0.0},
           {-3.191897893364441, 0.4216542921841766, 0.0}},
          {{1.3142186234817816, -0.5559962209319393, 0.0},
           {-2.421061140465244, 0.10899878597177906, 0.0}}};
      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          points,
          config);
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

      rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper, 0.1_psi};

      frc::TrajectoryConfig config(17.5_fps, 13_fps_sq);
      config.AddConstraint(m_drivetrain_constraint);

      std::vector<frc::Spline<5>::ControlVector> p1{
          {{0.9123293762437384, 1.0, 0.0},
           {-2.295882522473067, 0.0, 0.0}},
          {{1.5909292527276473, 0.5923188379890154, 0.0},
           {-2.2299990393192894, 0.22780070097757024, 0.0}},
          {{2.289294174157689, 0.01730569574621521, 0.0},
           {-1.0704497358128044, 0.6749221341023793, 0.0}}};

      std::vector<frc::Spline<5>::ControlVector> p2{
          {{2.2761174775269333, -0.008652847873107383, 0.0},
           {-1.08362643244356, 1.038341744772891, 0.0}},
          {{3.3762588320115356, -0.9950775054073548, 0.0},
           {-3.49739870223504, 0.5883936553713047, 0.0}},
          {{4.293460706560923, -0.8047148521989902, 0.0},
           {-3.49739870223504, -0.527823720259553, 0.0}},
          {{4.588627736224526, 0.0, 0.0},
           {-1.1692749605434711, -0.4018892472380433, 0.0}}};

      std::vector<frc::Spline<5>::ControlVector> p3{
          {{4.568862691278392, -0.1470984138428264, 0.0},
           {-1.004566252659027, -1.8863208363374186, 0.0}},
          {{5.054911319394376, 1.0693426360018272, 0.0},
           {-3.5579686373467916, -0.45498981121525406, 0.0}},
          {{6.430714131218457, 1.0834365779143758, 0.0},
           {-3.644497116077866, 0.3176780179667277, 0.0}},
          {{6.8681962533452285, 0.017305695746214766, 0.0},
           {-1.1033914773896933, 0.7787563085796683, 0.0}}};

      std::vector<frc::Spline<5>::ControlVector> p4{
          {{6.8681962533452285, -0.06056993511175168, 0.0},
           {-1.1363332189665822, 1.0037303532804613, 0.0}},
          {{7.454559253413847, -0.5952027949320489, 0.0},
           {-1.9664651067041792, 0.5291162912208613, 0.0}},
          {{8.811759006381665, -0.7015575628225967, 0.0},
           {-2.513298016880533, 0.19098313121751653, 0.0}}};

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

      m_drive.SetImpel(-1.0);

      auto reference = m_trajectory.Sample(m_autoTimer.Get());
      auto speeds = m_ramsete.Calculate(m_drive.GetPose(), reference);
      m_drive.Drive(speeds.vx, speeds.omega);

      wpi::json j;
      frc::to_json(j, reference);
      auto reference_json = j.dump(0);

      frc::SmartDashboard::PutString("Reference", reference_json);

      std::cout << reference.pose.X().value() << "," << reference.pose.Y().value() << "," << reference.pose.Rotation().Degrees().value() << "," << m_drive.GetPose().X().value() << "," << m_drive.GetPose().Y().value() << "," << m_drive.GetPose().Rotation().Degrees().value() << "," << speeds.vx.value() << "," << speeds.omega.value() << std::endl;
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
    double impelSpd = ((m_controller.GetRawAxis(3) / 2.0) + 0.5) - ((m_controller.GetRawAxis(4) / 2.0) + 0.5);
    m_drive.SetImpel(impelSpd);
    // m_drive.SetImpel(-1.0);
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

  string pose_json;
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
