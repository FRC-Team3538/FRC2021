#pragma once

#include <fstream>
#include <string>
#include <map>

using namespace std;

class Logging
{
  private:
    ofstream m_file;
    map<string, string> m_map;

    string m_path;
    string m_filename;

    bool m_started = false;

  public:
    Logging(string path = "/u/logs/", string filename = "log-#.csv");
    
    void AddKey(string key);
    void Start();
    void Log(string key, string value);
    void Commit();
};