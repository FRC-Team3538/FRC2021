#include "auto/AutoPrograms.hpp"

// Include all auto programs [List 1 of 3]
#include "auto/AutoEightBall.hpp"
#include "auto/AutoIntAcc.hpp"
#include "auto/AutoPowerPort.hpp"

// Constructor requires a reference to the robot map
AutoPrograms::AutoPrograms(robotmap &IO) : IO(IO)
{
    // SmartDash Chooser [List 2 of 3]
    m_chooser.SetDefaultOption("0 - None", "0 - None");
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
    m_autoProgram->Init();
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