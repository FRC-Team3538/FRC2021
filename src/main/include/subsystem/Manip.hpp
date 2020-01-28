#pragma once

#include <ctre/Phoenix.h>
#include <frc/PWMTalonSRX.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Solenoid.h>

using namespace frc;
using namespace ctre::phoenix::motorcontrol::can;

class Manip
{
  private:

    // PWM
   //PWMTalonSRX motorA1PWM;
    //PWMTalonSRX motorA2PWM{7};
    //PWMTalonSRX motorB1PWM{8};
    //PWMTalonSRX motorB2PWM{9};

    // CTRE CAN
    
    WPI_VictorSPX motorC {21};
    WPI_VictorSPX motorD {22};
    //WPI_TalonSRX motorB1 {8};
    //WPI_TalonSRX motorB2 {9};

    // Solenoids
    Solenoid sol1{1};
    Solenoid sol2{2};

  public:
    // Default Constructor
    Manip();

    void Stop();
    void SetA(double speed);
    void SetB(double speed);
    void SetC(double speed);
    void SetD(double speed);

    void SetSol0(bool state);
    void SetSol1(bool state);
    void ToggleSol2();

    void UpdateSmartdash();
};