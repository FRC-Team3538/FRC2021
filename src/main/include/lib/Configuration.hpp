#pragma once

#include <wpi/json.h>
#include <wpi/raw_ostream.h>

#include <fstream>
#include <iostream>
#include <string>

class Configuration
{
protected:
  // TODO: This should differentiate between Comp, Practice, etc environments.
  // This can be done file-wise by adding Config files under subfolders, as seen
  // in the example. Checking for /home/lvuser/THIS_IS_THE_COMP_BOT or the
  // alternative is how I've done it in the past.
  std::string get_path() { return "/home/lvuser/config/"; }

public:
  template<typename T>
  T Get(std::string file)
  {
    std::ifstream json_file(get_path() + file);
    if (json_file) {
      std::string contents;
      json_file.seekg(0, std::ios::end);
      contents.resize(json_file.tellg());
      json_file.seekg(0, std::ios::beg);
      json_file.read(&contents[0], contents.size());
      json_file.close();

      auto json_contents = wpi::json::parse(contents);
      return json_contents.get<T>();
    }

    throw errno;
  }

  template<typename T>
  T TryGet(std::string file)
  {
    try {
      return Get<T>(file);
    } catch (int i) {
      wpi::outs() << "Error while reading " << file << ": " << strerror(i)
                  << "\n";
    } catch (wpi::detail::exception e) {
      wpi::outs() << "Error while reading " << file << ": " << e.what() << "\n";
    }

    T alt;
    return alt;
  }

  template<typename T>
  void Put(std::string file, T data)
  {
    std::ofstream json_file(get_path() + file);
    if (json_file.is_open()) {
      std::string contents;
      wpi::json j = data;
      json_file << j.dump(4);
      json_file.close();
      return;
    }

    throw errno;
  }

  template<typename T>
  void TryPut(std::string file, T data)
  {
    try {
      Put<T>(file, data);
    } catch (int i) {
      wpi::outs() << "Error while writing to " << file << ": " << strerror(i)
                  << "\n";
    } catch (wpi::detail::exception e) {
      wpi::outs() << "Error while writing to " << file << ": " << e.what()
                  << "\n";
    }
  }
};