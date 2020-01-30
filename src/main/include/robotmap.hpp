#pragma once

#include <subsystem/DS.hpp>
#include <subsystem/Drivebase.hpp>
#include <subsystem/Shooter.hpp>
#include <subsystem/Climber.hpp>
#include <subsystem/ColorWheel.hpp>
#include <subsystem/RJVisionPipeline.hpp>
#include <subsystem/Logging.hpp>

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
};

