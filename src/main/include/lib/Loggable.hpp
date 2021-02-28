#pragma once

#include <lib/UDPLogger.hpp>

namespace rj {
class Loggable
{
public:
  virtual void Log(UDPLogger& logger) = 0;
};
}