#include "link/UartLink.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstdio>

#include "third_party/drone_link.h"

// NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
UartLink::UartLink(const std::string& dev)
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
  : fileDescriptor(open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK))
{
  if (fileDescriptor < 0) {
    perror("open uart");
    return;
  }

  termios tio{};
  tcgetattr(fileDescriptor, &tio);
  cfmakeraw(&tio);  // 8N1, без обробки символів
  cfsetispeed(&tio, B115200);
  cfsetospeed(&tio, B115200);  // швидкість з обох боків однакова!
  tio.c_cflag |= (CLOCAL | CREAD);
  tcsetattr(fileDescriptor, TCSANOW, &tio);
}

UartLink::~UartLink()
{
  if (fileDescriptor >= 0) {
    close(fileDescriptor);
  }
}

bool UartLink::isOpen() const
{
  return fileDescriptor >= 0;
}

int UartLink::readBytes(uint8_t* buf, int size) const
{
  return static_cast<int>(read(fileDescriptor, buf, static_cast<size_t>(size)));
}

void UartLink::sendControl(float accel, float turnRate) const
{
  const dlink::Control c{accel, turnRate};
  uint8_t out[64];
  const size_t m = dlink::encode(dlink::PKT_CONTROL, &c, sizeof c, out);

  if (write(fileDescriptor, out, m) < 0) {
    perror("Failed to send control by uart");
  }
}
// NOLINTEND(cppcoreguidelines-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
