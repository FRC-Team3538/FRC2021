#include "auto/AutoPrograms.hpp"

// Include all auto programs [List 1 of 3]
#include "auto/AutoLineCross.hpp"
#include "auto/AutoCenterShootForward.hpp"
#include "auto/AutoCenterShootBack.hpp"
#include "auto/AutoTrenchRun.hpp"
#include "auto/AutoBackTrench.hpp"
#include "auto/AutoStealTrenchRun.hpp"
#include "auto/AutoShootTrenchYolo.hpp"
#include "auto/AutoEightBall.hpp"
#include "auto/AutoIntAcc.hpp"
#include "auto/AutoPowerPort.hpp"

// Constructor requires a reference to the robot map
AutoPrograms::AutoPrograms(robotmap &IO) : IO(IO)
{
    // SmartDash Chooser [List 2 of 3]
    m_chooser.SetDefaultOption("0 - None", "0 - None");
    m_chooser.AddOption(AutoLineCross::GetName(), AutoLineCross::GetName());
    m_chooser.AddOption(AutoCenterShootForward::GetName(), AutoCenterShootForward::GetName());
    m_chooser.AddOption(AutoCenterShootBack::GetName(), AutoCenterShootBack::GetName());
    m_chooser.AddOption(AutoTrenchRun::GetName(), AutoTrenchRun::GetName());
    m_chooser.AddOption(AutoBackTrench::GetName(), AutoBackTrench::GetName());
    m_chooser.AddOption(AutoStealTrenchRun::GetName(), AutoStealTrenchRun::GetName());
    m_chooser.AddOption(AutoShootTrenchYolo::GetName(), AutoShootTrenchYolo::GetName());
    m_chooser.AddOption(AutoEightBall::GetName(), AutoEightBall::GetName());
    m_chooser.AddOption(AutoIntAcc::GetName(), AutoIntAcc::GetName());
    m_chooser.AddOption(AutoPowerPort::GetName(), AutoPowerPort::GetName());
}

// Initialize the selected auto program
void AutoPrograms::Init()
{
    // Get Selected Program from SmartDash Chooser
    std::string name = m_chooser.GetSelected();

    // Delete previously selected auto program
    delete m_autoProgram;
    m_autoProgram = NULL;

    // Create the Selected auto program [List 3 of 3]
    if (name == AutoLineCross::GetName())
    {
        m_autoProgram = new AutoLineCross(IO);
    }
    if (name == AutoCenterShootForward::GetName())
    {
        m_autoProgram = new AutoCenterShootForward(IO);
    }
    if (name == AutoCenterShootBack::GetName())
    {
        m_autoProgram = new AutoCenterShootBack(IO);
    }
    if (name == AutoTrenchRun::GetName())
    {
        m_autoProgram = new AutoTrenchRun(IO);
    }
    if (name == AutoBackTrench::GetName())
    {
        m_autoProgram = new AutoTrenchRun(IO);
    }
    if (name == AutoStealTrenchRun::GetName())
    {
        m_autoProgram = new AutoStealTrenchRun(IO);
    }
    if (name == AutoShootTrenchYolo::GetName())
    {
        m_autoProgram = new AutoShootTrenchYolo(IO);
    }
    if (name == AutoEightBall::GetName())
    {
        m_autoProgram = new AutoEightBall(IO);
    }
    if (name == AutoIntAcc::GetName())
    {
        m_autoProgram = new AutoIntAcc(IO);
    }
    if (name == AutoPowerPort::GetName())
    {
        m_autoProgram = new AutoPowerPort(IO);
    }
}

// Run the selected Auto Program
void AutoPrograms::Run()
{
    if (m_autoProgram)
    {
        m_autoProgram->Run();
    }
}

void AutoPrograms::SmartDash()
{
    SmartDashboard::PutData("Choose Auto", &m_chooser);
    std::string name = m_chooser.GetSelected();
    SmartDashboard::PutString("Selected Auto", name);
}