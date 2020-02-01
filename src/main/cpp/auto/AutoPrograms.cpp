#include "auto/AutoPrograms.hpp"

// Include all auto programs [List 1 of 3]
#include "auto/AutoLineCross.hpp"
#include "auto/AutoCenterShootForward.hpp"
#include "auto/AutoCenterShootBack.hpp"
#include "auto/AutoTrenchRun.hpp"

// Constructor requires a reference to the robot map
AutoPrograms::AutoPrograms(robotmap &IO) : IO(IO)
{
    // SmartDash Chooser [List 2 of 3]
    m_chooser.SetDefaultOption("0 - None", "0 - None");
    m_chooser.AddOption(AutoLineCross::GetName(), AutoLineCross::GetName());
    m_chooser.AddOption(AutoCenterShootForward::GetName(), AutoCenterShootForward::GetName());
    m_chooser.AddOption(AutoCenterShootBack::GetName(), AutoCenterShootBack::GetName());
    m_chooser.AddOption(AutoTrenchRun::GetName(), AutoTrenchRun::GetName());
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