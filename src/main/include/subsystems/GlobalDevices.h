#pragma once

#include <lib/Loggable.hpp>
#include <frc/PowerDistributionPanel.h>
#include <frc/Compressor.h>


class GlobalDevices: public rj::Loggable
{
  frc::PowerDistributionPanel pdp;
  // frc::Compressor pcm;
    
  void Log(UDPLogger& logger)
  {
    logger.LogExternalDevice(pdp);
    // logger.LogExternalDevice(pcm);
  }
};