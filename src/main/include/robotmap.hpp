#pragma once

#include <subsystem/DS.hpp>
#include <subsystem/Drivebase.hpp>
#include <subsystem/Manip.hpp>

class robotmap
{
  public:
    DS ds;
    Drivebase drivebase;
    Manip manip;
};