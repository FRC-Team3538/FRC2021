#include "subsystem/Manip.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

// Configure Hardware Settings
Manip::Manip()
{
    
}

// Stop all motors
void Manip::Stop()
{
    motorGroupA.StopMotor();
}

void Manip::SetA(double speed)
{
   motorGroupA.Set(speed);
}

void Manip::SetB(double speed)
{
   motorGroupB.Set(speed);
}

void Manip::SetSol1(bool state)
{
   sol1.Set(state);
}

void Manip::ToggleSol2()
{
   sol2.Set(!sol2.Get());
}


void Manip::UpdateSmartdash()
{
    SmartDashboard::PutNumber("Motor A", motorGroupA.Get());
    SmartDashboard::PutNumber("Motor B", motorGroupB.Get());
    SmartDashboard::PutBoolean("Solenoid A", sol1.Get());
    SmartDashboard::PutBoolean("Solenoid B", sol2.Get());
}
