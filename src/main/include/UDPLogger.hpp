#pragma once

#include <functional>
#include <mutex>
#include <vector>

#if defined(_WIN32)

#else
#include <netinet/in.h>
#endif // defined(_WIN32)

#include "flatbuffers/flatbuffers.h"

using namespace std;

#define FLATBUFFER_SIZE 4096

class UDPLogger
{
private:
  flatbuffers::FlatBufferBuilder fbb{ FLATBUFFER_SIZE };
  uint8_t buf[FLATBUFFER_SIZE]; // 4KB
  size_t bufsize;

  int sockfd;
#if defined(_WIN32)
#else
  struct sockaddr_in address;
  std::vector<struct sockaddr_in> clients;
#endif // defined(_WIN32)
  std::recursive_mutex mut;
  std::string title;

public:
  void InitLogger();
  void CheckForNewClient();
  void FlushLogBuffer();
  void Log(uint8_t* data, size_t size);
  void SetTitle(std::string str);
};
