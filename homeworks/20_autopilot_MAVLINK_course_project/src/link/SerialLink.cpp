#include "link/SerialLink.hpp"
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <array>
#include <cerrno>
#include <cstring>
#include <utility>

#include "Logger.hpp"

namespace {

constexpr int writeWaitMs = 100;  // скільки максимум чекаємо поки звільниться буфер виходу

constexpr std::array<std::pair<int, speed_t>, 8> supportedSpeeds{{
  {9600, B9600},
  {19200, B19200},
  {38400, B38400},
  {57600, B57600},
  {115200, B115200},
  {230400, B230400},
  {460800, B460800},
  {921600, B921600},
}};

bool toSpeed(const int baudRate, speed_t& speed)
{
  for (const auto& [supportedBaudRate, supportedSpeed] : supportedSpeeds) {
    if (supportedBaudRate == baudRate) {
      speed = supportedSpeed;
      return true;
    }
  }
  return false;
}

}  // namespace

// NOLINTBEGIN(cppcoreguidelines-pro-type-vararg,cppcoreguidelines-pro-bounds-pointer-arithmetic)
SerialLink::SerialLink(const std::string& device, const int baudRate)
  // відкривання порту на читання та запис, без керівного терміналу, читання неблокуюче
  : fileDescriptor(::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK))
{
  if (fileDescriptor < 0) {
    LOG("serial: open '" << device << "' failed: " << strerror(errno));
    return;
  }

  speed_t speed = 0;
  if (!toSpeed(baudRate, speed)) {
    LOG("serial: unsupported baud rate " << baudRate);
    cleanup();
    return;
  }

  termios portSettings{};
  if (tcgetattr(fileDescriptor, &portSettings) != 0) {
    LOG("serial: tcgetattr failed: " << strerror(errno));
    cleanup();
    return;
  }

  cfmakeraw(&portSettings);                  // вимкнення обробки тексту
  portSettings.c_cflag |= (CLOCAL | CREAD);  // прямий дріт без модему, приймач ввімкнули
  portSettings.c_cflag &= ~CSTOPB;           // 1 стоп-біт, 8N1 разом із cfmakeraw
  portSettings.c_cflag &= ~CRTSCTS;          // вимкнення апаратного керування потоком

  // Керування тим коли read() повертає результат
  portSettings.c_cc[VMIN] = 1;   // Чекаємо хоча б один байт
  portSettings.c_cc[VTIME] = 0;  // без таймеру очікування

  // Задаємо швидкість читання та запису
  // має бути однакова не тільки між собою, а ще з пристроєм на іншому кінці
  cfsetispeed(&portSettings, speed);  // швидкість на вхід
  cfsetospeed(&portSettings, speed);  // швидкість на вихід

  // Застосування налаштувань portSettings до самого порту
  if (tcsetattr(fileDescriptor, TCSANOW, &portSettings) != 0) {
    LOG("serial: tcsetattr failed: " << strerror(errno));
    cleanup();
    return;
  }

  // Очистити потенційне сміття яке могло утворитись при завантаженні
  tcflush(fileDescriptor, TCIOFLUSH);
}

SerialLink::~SerialLink()
{
  if (fileDescriptor >= 0) {
    close(fileDescriptor);
  }
}

bool SerialLink::isOpen() const
{
  return fileDescriptor >= 0;
}

void SerialLink::sendFrame(const uint8_t* buf, const size_t len) const
{
  size_t written = 0;  // скільки вже відправлено байтів

  while (written < len) {
    // скільки відправлено байтів у цій ітерації
    const ssize_t bytesWritten = ::write(fileDescriptor,
                                         buf + written,  // де у буфері шукати байти
                                         len - written   // скільки ще лишилось байтів відправити
    );

    if (bytesWritten > 0) {
      written += static_cast<size_t>(bytesWritten);
      continue;
    }

    // Перевірка на переривання сигналом, якщо відправку було перервано системою
    // безпечно продовжимо на наступній ітерації
    if (bytesWritten < 0 && errno == EINTR) {
      continue;
    }

    // Перевірка чи заповнений буфер повністю
    if (bytesWritten < 0 && errno == EAGAIN) {
      // Запит для poll, за яким fd стежити, якою подією, і код результату (буде перезаписано)
      pollfd waitForOutput{.fd = fileDescriptor, .events = POLLOUT, .revents = 0};

      // "приспати" програму доки порт не звільниться (на writeWaitMs часу) і можна буде знову писати без блокування (подія POLLOUT)
      if (poll(&waitForOutput, 1, writeWaitMs) <= 0) {
        LOG("serial send timed out, " << (len - written) << " of " << len << " bytes lost");
        return;
      }
      continue;
    }

    LOG("serial send failed: " << strerror(errno));
    return;
  }
}

ssize_t SerialLink::receive(uint8_t* buf, const size_t capacity) const
{
  const ssize_t bytesRead = ::read(fileDescriptor, buf, capacity);
  if (bytesRead < 0) {
    // Перевірка на те чи пустий буфер, або чи не було перервано читання сигналом системи
    return (errno == EAGAIN || errno == EINTR) ? -1 : 0;
  }
  return bytesRead;
}

void SerialLink::cleanup()
{
  close(fileDescriptor);
  fileDescriptor = -1;
}
// NOLINTEND(cppcoreguidelines-pro-type-vararg,cppcoreguidelines-pro-bounds-pointer-arithmetic)
