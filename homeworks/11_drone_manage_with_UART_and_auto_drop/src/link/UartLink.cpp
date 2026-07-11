#include "link/UartLink.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstdio>

#include "third_party/drone_link.h"

// NOLINTBEGIN(cppcoreguidelines-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
UartLink::UartLink(const std::string& dev)
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
  : fd(open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK))
{
  if (fd < 0) {
    perror("open uart");
    return;
  }

  termios tio{};
  tcgetattr(fd, &tio);
  cfmakeraw(&tio);  // 8N1, без обробки символів
  cfsetispeed(&tio, B115200);
  cfsetospeed(&tio, B115200);  // швидкість з обох боків однакова!
  tio.c_cflag |= (CLOCAL | CREAD);
  tcsetattr(fd, TCSANOW, &tio);
}

UartLink::~UartLink()
{
  if (fd >= 0) {
    close(fd);
  }
}

bool UartLink::isOpen() const
{
  return fd >= 0;
}

int UartLink::readBytes(uint8_t* buf, int size) const
{
  return static_cast<int>(read(fd, buf, static_cast<size_t>(size)));
}

void UartLink::sendControl(float accel, float turnRate)
{
  const dlink::Control c{accel, turnRate};
  uint8_t out[64];
  const size_t m = dlink::encode(dlink::PKT_CONTROL, &c, sizeof c, out);

  if (write(fd, out, m) < 0) {
    perror("write uart");
  }
}
// NOLINTEND(cppcoreguidelines-avoid-c-arrays,cppcoreguidelines-pro-bounds-array-to-pointer-decay)
