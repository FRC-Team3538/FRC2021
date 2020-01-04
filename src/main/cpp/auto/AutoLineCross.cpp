#include "auto/AutoLineCross.hpp"

// Name for Smart Dash Chooser
std::string AutoLineCross::GetName()
{
    return "1 - AutoLineCross";
}

// Initialization
// Constructor requires a reference to the robot map
AutoLineCross::AutoLineCross(robotmap &IO) : IO(IO)
{
    m_state = 0;
    m_autoTimer.Reset();
    m_autoTimer.Start();
    IO.drivebase.Stop();
}

AutoLineCross::~AutoLineCross() { }


//State Machine
void AutoLineCross::NextState(){
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

// Execute the program
void AutoLineCross::Run()
{
    switch (m_state)
    {
    case 0:
    {
        double fwd = 0.50;
        double rot = 0.00;
        IO.drivebase.Arcade(fwd, rot);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    case 1:
    {
        double fwd = 0.00;
        double rot = 0.25;
        IO.drivebase.Arcade(fwd, rot);
        if (m_autoTimer.Get() > 2.0)
        {
            NextState();
        }
        break;
    }
    default:
        IO.drivebase.Stop();
    }
}