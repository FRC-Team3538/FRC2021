#include "subsystem/Manip.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

// Configure Hardware Settings
Manip::Manip()
{
   SmartDashboard::PutNumber("Motor A Max", 1.0);
   SmartDashboard::PutNumber("Motor B Max", 1.0);
   SmartDashboard::PutNumber("Motor C Max", 1.0);
   SmartDashboard::PutNumber("Motor D Max", 1.0);
}

// Stop all motors
void Manip::Stop()
{
    motorA.StopMotor();
    motorB.StopMotor();
    motorC.StopMotor();
    motorD.StopMotor();
}

void Manip::SetA(double speed)
{
   double max = SmartDashboard::GetNumber("Motor A Max", 1.0);
   motorA.Set(speed * max);
}

void Manip::SetB(double speed)
{
   double max = SmartDashboard::GetNumber("Motor B Max", 1.0);
   motorB.Set(speed * max);
}

void Manip::SetC(double speed)
{
    double max = SmartDashboard::GetNumber("Motor C Max", 1.0);
    motorC.Set(speed * max);
}

void Manip::SetD(double speed)
{
    double max = SmartDashboard::GetNumber("Motor D Max", 1.0);
    motorD.Set(speed * max);
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
    SmartDashboard::PutNumber("Motor A", motorA.Get());
    SmartDashboard::PutNumber("Motor B", motorB.Get());
    SmartDashboard::PutNumber("Motor C", motorC.Get());
    SmartDashboard::PutNumber("Motor D", motorD.Get());
    SmartDashboard::PutBoolean("Solenoid A", sol1.Get());
    SmartDashboard::PutBoolean("Solenoid B", sol2.Get());
}
