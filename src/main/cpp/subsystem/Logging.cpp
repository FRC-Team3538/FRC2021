#include "subsystem/Logging.hpp"

#include <iostream>

#include "frc/Timer.h"
#include "frc/Preferences.h"


Logging::Logging(string path, string filename)
{
    // Default Values
    if(path == "") path = "/u/logs/";
    if(filename == "") filename = "log-#.csv";

    // Get a log serial number from robot Preferences
    auto pref = frc::Preferences::GetInstance();
    auto logNumber = pref->GetInt("LogNumber", 1);
    pref->PutInt("LogNumber", (logNumber + 1) % 200 ); // Limit to 200 log files

    // pad with zeros
    auto logNumberStr = to_string(logNumber);
    while (logNumberStr.length() < 6)
    {
        logNumberStr = "0" + logNumberStr;
    }

    // Replace # in the log file name with the log number
    auto i = filename.find("#");
    if (i >= 0)
    {
        filename = filename.replace(i, 1, logNumberStr, 0, logNumberStr.length());
    }

    m_path = path;
    m_filename = filename;
}

void Logging::AddKey(string _key)
{
    // Cannot add new columns once logging is started
    if (m_started)
        return;

    m_map[_key] = "NULL";
}

void Logging::Start()
{
    // Cannot add new columns once logging is started
    if (m_started)
        return;

    // Cannot add new columns once logging is started
    m_started = true;

    // Make directory if it doesn't exist
    // TODO

    // Lets open dat log file!
    m_file.open(m_path + m_filename);

    cout << "Open: " << m_path << m_filename << endl;

    // Print the header
    m_file << "Time, ";

    // Print each log item
    for (auto &i : m_map)
    {
        m_file << i.first << ", ";
    }

    // Force output to disk
    m_file << endl;
    m_file.flush();
}

void Logging::Log(string _key, string _value)
{
    // Must start log first
    if (!m_started)
    {
        return;
    }

    // Only update registered items
    if (m_map.count(_key) == 0)
    {
        return;
    }

    m_map[_key] = _value;
}

void Logging::Commit()
{
    // Must start log first
    if (!m_started)
    {
        return;
    }

    m_file << frc::Timer::GetFPGATimestamp() << ", ";

    for (auto &i : m_map)
    {
        m_file << i.second << ", ";
    }

    // Force output to disk
    m_file << endl;
    m_file.flush();
}