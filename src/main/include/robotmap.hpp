#pragma once

#include <ExternalDeviceProvider.hpp>
#include <UDPLogger.hpp>
#include <subsystem/Climber.hpp>
#include <subsystem/ColorWheel.hpp>
#include <subsystem/DS.hpp>
#include <subsystem/Drivebase.hpp>
#include <subsystem/Logging.hpp>
#include <subsystem/RJVisionPipeline.hpp>
#include <subsystem/Shooter.hpp>

class robotmap
{
public:
  UDPLogger logger;
  ExternalDeviceProvider externalDeviceProvider;

  DS ds;
  Drivebase drivebase;
  Shooter shooter;
  Climber climber;
  ColorWheel colorWheel;
  vision::RJVisionPipeline RJV;
  Logging log;
};
