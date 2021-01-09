#include <iostream>

#if defined(_WIN32)

#else
#include <arpa/inet.h>
#include <netinet/ip.h>
#include <sys/socket.h>
#endif // defined(_WIN32)

#include "frc/Timer.h"

#include "UDPLogger.hpp"
#include "proto/StatusFrame_generated.h"
#include <assert.h>

#include <iostream>

#if defined(_WIN32)

void
UDPLogger::InitLogger()
{}

void
UDPLogger::CheckForNewClient()
{}

void
UDPLogger::FlushLogBuffer()
{}

#else

void
UDPLogger::InitLogger()
{

  sockfd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, IPPROTO_UDP);
  if (sockfd < 0) {
    std::cout << "socket() failed! " << strerror(errno) << std::endl;
    return;
  }

  address.sin_family = AF_INET;
  address.sin_port = htons(3538);

  if (inet_aton("0.0.0.0", &address.sin_addr) == 0) {
    std::cout << "inet_aton() failed! " << strerror(errno) << std::endl;
    return;
  }

  if (bind(sockfd, (const struct sockaddr*)&address, sizeof(address)) != 0) {
    std::cout << "bind() failed! " << strerror(errno) << std::endl;
    return;
  }
}

int
sendLog(int sockfd,
        const uint8_t* data,
        size_t size,
        const struct sockaddr_in& address)
{
  if (sockfd < 0) {
    return 0;
  }

  return sendto(
    sockfd, data, size, 0, (const struct sockaddr*)&address, sizeof(address));
}

// Need to lock at this level so we don't cause iterator invalidation on
// `clients`
void
UDPLogger::FlushLogBuffer()
{
  mut.lock();
  for (struct sockaddr_in addr : clients) {
    if (sendLog(sockfd, buf, bufsize, addr) == -1) {
      std::cout << "sendLog failed! " << strerror(errno) << std::endl;
    }
  }
  bufsize = 0;
  mut.unlock();
}

#define RECV_BUF_SIZE 3

void
UDPLogger::CheckForNewClient()
{
  struct sockaddr_in client;
  socklen_t client_len = sizeof(struct sockaddr_in);
  char buf[RECV_BUF_SIZE];
  buf[2] = 0x00;
  ssize_t res_len = recvfrom(sockfd,
                             (void*)buf,
                             RECV_BUF_SIZE,
                             0,
                             (struct sockaddr*)&client,
                             &client_len);

  if (res_len == 2 && strcmp(buf, "Hi") == 0) {
    mut.lock();
    clients.push_back(client);

    fbb.Reset();
    auto greeting = rj::CreateInitializeStatusFrameDirect(fbb, title.c_str());
    auto wrapper =
      rj::CreateStatusFrameHolder(fbb,
                                  frc::GetTime(),
                                  frc::Timer::GetFPGATimestamp(),
                                  rj::StatusFrame_InitializeStatusFrame,
                                  greeting.Union());
    fbb.FinishSizePrefixed(wrapper);
    auto buffer = fbb.Release();

    sendLog(sockfd, buffer.data(), buffer.size(), client);

    mut.unlock();
  }
}

#endif // defined(_WIN32)

void
UDPLogger::Log(uint8_t* data, size_t size)
{
  // shoutouts to memory safety
  assert(bufsize + size < FLATBUFFER_SIZE);
  memcpy(buf + bufsize, data, size);
  bufsize += size;
}

void
UDPLogger::SetTitle(std::string str)
{
  title = str;
}