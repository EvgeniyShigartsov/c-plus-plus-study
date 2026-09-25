#include "link/UdpLink.hpp"
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

#include "Logger.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-type-vararg,cppcoreguidelines-pro-type-reinterpret-cast)
UdpLink::UdpLink(const std::string& host, const uint16_t port, const uint16_t ownPort)
  : fileDescriptor(socket(AF_INET, SOCK_DGRAM, 0))
{
  if (fileDescriptor < 0) {
    LOG("udp: socket failed: " << strerror(errno));
    return;
  }

  const int flags = fcntl(fileDescriptor, F_GETFL, 0);
  fcntl(fileDescriptor, F_SETFL, flags | O_NONBLOCK);

  if (ownPort != 0) {
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(ownPort);
    local.sin_addr.s_addr = INADDR_ANY;

    const int bindResult = bind(fileDescriptor, reinterpret_cast<sockaddr*>(&local), sizeof local);
    if (bindResult < 0) {
      LOG("udp: bind failed: " << strerror(errno));
      cleanup();
      return;
    }
  }

  destination.sin_family = AF_INET;
  destination.sin_port = htons(port);

  if (inet_pton(AF_INET, host.c_str(), &destination.sin_addr) != 1) {
    LOG("udp: bad host '" << host << "'");
    cleanup();
  }
}

UdpLink::~UdpLink()
{
  if (fileDescriptor >= 0) {
    close(fileDescriptor);
  }
}

bool UdpLink::isOpen() const
{
  return fileDescriptor >= 0;
}

void UdpLink::sendFrame(const uint8_t* buf, const size_t len) const
{
  if (::sendto(fileDescriptor, buf, len, 0, reinterpret_cast<const sockaddr*>(&destination), sizeof destination) < 0) {
    if (errno != ECONNREFUSED && errno != EAGAIN) {
      LOG("udp send failed: " << strerror(errno));
    }
  }
}

ssize_t UdpLink::receive(uint8_t* buf, const size_t capacity) const
{
  const ssize_t bytesRead = ::recvfrom(fileDescriptor, buf, capacity, 0, nullptr, nullptr);
  if (bytesRead == -1) {
    return (errno == EAGAIN || errno == EWOULDBLOCK) ? -1 : 0;
  }
  return bytesRead;
}

void UdpLink::cleanup()
{
  close(fileDescriptor);
  fileDescriptor = -1;
}

// NOLINTEND(cppcoreguidelines-pro-type-vararg,cppcoreguidelines-pro-type-reinterpret-cast)