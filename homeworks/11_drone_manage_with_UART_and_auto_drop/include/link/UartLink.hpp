#pragma once
#include <cstdint>
#include <string>

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class UartLink {
public:
  UartLink(const std::string& dev);
  ~UartLink();

  bool isOpen() const;

  int readBytes(uint8_t* buf, int size) const;

  void sendControl(float accel, float turnRate) const;

private:
  int fileDescriptor = -1;
};
