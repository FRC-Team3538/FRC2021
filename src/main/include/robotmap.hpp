#pragma once

#include <UDPLogger.hpp>
#include <subsystem/Climber.hpp>
#include <subsystem/ColorWheel.hpp>
#include <subsystem/DS.hpp>
#include <subsystem/Drivebase.hpp>
#include <subsystem/GlobalDevices.hpp>
#include <subsystem/Logging.hpp>
#include <subsystem/RJVisionPipeline.hpp>
#include <subsystem/Shooter.hpp>

#include <lib/Configuration.hpp>
#include <lib/Loggable.hpp>

class RobotMap
{
private:
  Configuration config;
  std::string drivetrainFile { "DrivebaseConfig.json" };
  std::string shooterFile { "ShooterConfig.json" };
  std::string climberFile { "ClimberConfig.json" };
  std::string colorWheelFile { "ColorWheelConfig.json" };

public:
  UDPLogger logger;

  GlobalDevices globals;

  DS ds;
  Drivebase drivebase{};
  Shooter shooter{};
  Climber climber{};
  ColorWheel colorWheel{};
  vision::RJVisionPipeline RJV;
  Logging log;

  std::vector<std::shared_ptr<rj::Loggable>> loggables{
    std::shared_ptr<rj::Loggable>(&globals),
    std::shared_ptr<rj::Loggable>(&drivebase),
    std::shared_ptr<rj::Loggable>(&shooter),
    std::shared_ptr<rj::Loggable>(&climber),
    std::shared_ptr<rj::Loggable>(&colorWheel),
  };
};
