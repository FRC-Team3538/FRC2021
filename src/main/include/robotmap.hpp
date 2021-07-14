#pragma once

#include <subsystem/DS.hpp>
#include <subsystem/Drivebase.hpp>
#include <subsystem/Shooter.hpp>
#include <subsystem/Climber.hpp>
#include <subsystem/ColorWheel.hpp>
#include <subsystem/RJVisionPipeline.hpp>
#include <subsystem/Logging.hpp>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <opencv2/videoio.hpp>
#include <frc/PowerDistributionPanel.h>
#include <frc/Timer.h>
#include <frc/controller/RamseteController.h>
#include <frc/Preferences.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>
#include <wpi/json.h>
#include <lib/csv.h>
#include "subsystem/AutoConstants.hpp"

class robotmap
{
public:
  DS ds;
  Drivebase drivebase;
  Shooter shooter;
  Climber climber;
  ColorWheel colorWheel;
  vision::RJVisionPipeline RJV;
  Logging log;
  frc::RamseteController m_ramsete{kRamseteB, kRamseteZeta};
};
