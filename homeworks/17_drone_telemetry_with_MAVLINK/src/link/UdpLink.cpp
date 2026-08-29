#include "link/UdpLink.hpp"
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

#include "Logger.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-type-vararg,cppcoreguidelines-pro-type-reinterpret-cast)
UdpLink::UdpLink(const std::string& host, const uint16_t port)
  : fileDescriptor(socket(AF_INET, SOCK_DGRAM, 0))
{
  if (fileDescriptor < 0) {
    LOG("udp: socket failed: " << strerror(errno));
    return;
  }

  const int flags = fcntl(fileDescriptor, F_GETFL, 0);
  fcntl(fileDescriptor, F_SETFL, flags | O_NONBLOCK);

  sockaddr_in dest{};
  dest.sin_family = AF_INET;
  dest.sin_port = htons(port);

  if (inet_pton(AF_INET, host.c_str(), &dest.sin_addr) != 1) {
    LOG("udp: bad host '" << host << "'");
    close(fileDescriptor);
    fileDescriptor = -1;
    return;
  }

  if (connect(fileDescriptor, reinterpret_cast<sockaddr*>(&dest), sizeof dest) < 0) {
    LOG("udp: connect failed: " << strerror(errno));
    close(fileDescriptor);
    fileDescriptor = -1;
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
  if (::send(fileDescriptor, buf, len, 0) < 0) {
    if (errno != ECONNREFUSED && errno != EAGAIN) {
      LOG("udp send failed: " << strerror(errno));
    }
  }
}

int UdpLink::receive(uint8_t* buf, const size_t capacity) const
{
  const ssize_t bytesRead = ::recv(fileDescriptor, buf, capacity, 0);
  if (bytesRead == -1) {
    return (errno == EAGAIN || errno == EWOULDBLOCK) ? -1 : 0;
  }
  return static_cast<int>(bytesRead);
}
// NOLINTEND(cppcoreguidelines-pro-type-vararg,cppcoreguidelines-pro-type-reinterpret-cast)
