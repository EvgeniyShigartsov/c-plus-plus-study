#include <gtest/gtest.h>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include "link/SerialLink.hpp"
#include "mavlink/Codec.hpp"
#include "mavlink/Endpoint.hpp"

namespace {

constexpr int baudRate = 115200;
constexpr std::chrono::milliseconds waitBudget = std::chrono::milliseconds(500);

class PtyPair {
public:
  PtyPair()
    : master(posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK))
  {
    if (master < 0 || grantpt(master) != 0 || unlockpt(master) != 0) {
      return;
    }
    const char* name = ptsname(master);
    if (name != nullptr) {
      slavePath = name;
    }

    termios tty{};
    tcgetattr(master, &tty);
    cfmakeraw(&tty);
    tcsetattr(master, TCSANOW, &tty);
  }

  ~PtyPair() { closeMaster(); }

  PtyPair(const PtyPair&) = delete;
  PtyPair& operator=(const PtyPair&) = delete;
  PtyPair(PtyPair&&) = delete;
  PtyPair& operator=(PtyPair&&) = delete;

  [[nodiscard]] bool isReady() const { return master >= 0 && !slavePath.empty(); }

  [[nodiscard]] const std::string& slave() const { return slavePath; }

  void writeToLink(const uint8_t* data, const size_t length) const { ASSERT_EQ(write(master, data, length), static_cast<ssize_t>(length)); }

  // Прочитати з "дроту" рівно length байтів (чекаємо, доки SerialLink їх запише)
  [[nodiscard]] std::vector<uint8_t> readFromLink(const size_t length) const
  {
    std::vector<uint8_t> result;
    std::array<uint8_t, 256> chunk{};
    const auto deadline = std::chrono::steady_clock::now() + waitBudget;

    while (result.size() < length && std::chrono::steady_clock::now() < deadline) {
      const ssize_t n = read(master, chunk.data(), chunk.size());
      if (n > 0) {
        result.insert(result.end(), chunk.begin(), chunk.begin() + n);
      }
      else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    return result;
  }

  void closeMaster()
  {
    if (master >= 0) {
      close(master);
      master = -1;
    }
  }

private:
  int master;
  std::string slavePath;
};

// Чекаємо, доки receive() щось поверне (дані через pty приходять не миттєво)
ssize_t receiveWithin(const SerialLink& link, uint8_t* buf, const size_t capacity)
{
  const auto deadline = std::chrono::steady_clock::now() + waitBudget;
  ssize_t n = -1;
  while (n == -1 && std::chrono::steady_clock::now() < deadline) {
    n = link.receive(buf, capacity);
    if (n == -1) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  return n;
}

}  // namespace

TEST(SerialLink, OpensPtyAndReportsOpen)
{
  const PtyPair pty;
  ASSERT_TRUE(pty.isReady());

  const SerialLink link(pty.slave(), baudRate);

  EXPECT_TRUE(link.isOpen());
}

TEST(SerialLink, MissingDeviceIsNotOpen)
{
  const SerialLink link("/dev/does-not-exist-hm20", baudRate);

  EXPECT_FALSE(link.isOpen());
}

TEST(SerialLink, UnsupportedBaudRateIsNotOpen)
{
  const PtyPair pty;
  ASSERT_TRUE(pty.isReady());

  const SerialLink link(pty.slave(), 12345);

  EXPECT_FALSE(link.isOpen());
}

TEST(SerialLink, SendFrameDeliversAllBytesToTheOtherEnd)
{
  const PtyPair pty;
  ASSERT_TRUE(pty.isReady());
  const SerialLink link(pty.slave(), baudRate);

  // Байти, які в "cooked"-режимі були б перетворені (CR, NL, Ctrl+C), мають пройти без змін
  const std::array<uint8_t, 8> frame{0xFD, 0x0D, 0x0A, 0x03, 0x00, 0x1A, 0xFF, 0x7F};
  link.sendFrame(frame.data(), frame.size());

  const std::vector<uint8_t> received = pty.readFromLink(frame.size());
  EXPECT_EQ(received, std::vector<uint8_t>(frame.begin(), frame.end()));
}

TEST(SerialLink, ReceiveReturnsBytesWrittenByTheOtherEnd)
{
  const PtyPair pty;
  ASSERT_TRUE(pty.isReady());
  const SerialLink link(pty.slave(), baudRate);

  const std::array<uint8_t, 5> frame{0x01, 0x0D, 0x0A, 0x03, 0xFE};
  pty.writeToLink(frame.data(), frame.size());

  std::array<uint8_t, 32> buffer{};
  const ssize_t n = receiveWithin(link, buffer.data(), buffer.size());

  ASSERT_EQ(n, static_cast<ssize_t>(frame.size()));
  EXPECT_TRUE(std::equal(frame.begin(), frame.end(), buffer.begin()));
}

TEST(SerialLink, ReceiveReturnsMinusOneWhenNothingToRead)
{
  const PtyPair pty;
  ASSERT_TRUE(pty.isReady());
  const SerialLink link(pty.slave(), baudRate);

  std::array<uint8_t, 32> buffer{};

  EXPECT_EQ(link.receive(buffer.data(), buffer.size()), -1);
}

TEST(SerialLink, ReceiveReturnsZeroWhenOtherEndIsGone)
{
  PtyPair pty;
  ASSERT_TRUE(pty.isReady());
  const SerialLink link(pty.slave(), baudRate);

  pty.closeMaster();

  std::array<uint8_t, 32> buffer{};
  EXPECT_EQ(link.receive(buffer.data(), buffer.size()), 0);
}

// Кадр MAVLink може прийти по UART шматками - парсер має дочекатись решти, а не загубити початок
TEST(SerialLink, EndpointAssemblesFrameSplitAcrossReads)
{
  const PtyPair pty;
  ASSERT_TRUE(pty.isReady());
  const SerialLink link(pty.slave(), baudRate);
  const mav::MavlinkEndpoint endpoint(link, MAVLINK_COMM_3);

  const mavlink_message_t heartbeat = mav::pack_heartbeat(mav::kVehicle, {.type = MAV_TYPE_QUADROTOR, .system_status = MAV_STATE_ACTIVE});
  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> frame{};
  const uint16_t length = mavlink_msg_to_send_buffer(frame.data(), &heartbeat);
  ASSERT_GT(length, 6);

  const size_t firstPart = 6;
  pty.writeToLink(frame.data(), firstPart);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_TRUE(endpoint.poll().empty());

  pty.writeToLink(frame.data() + firstPart, length - firstPart);

  std::vector<mavlink_message_t> messages;
  const auto deadline = std::chrono::steady_clock::now() + waitBudget;
  while (messages.empty() && std::chrono::steady_clock::now() < deadline) {
    messages = endpoint.poll();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  ASSERT_EQ(messages.size(), 1U);
  EXPECT_EQ(messages[0].msgid, MAVLINK_MSG_ID_HEARTBEAT);
  EXPECT_EQ(mav::parse_heartbeat(messages[0]).system_status, MAV_STATE_ACTIVE);
}

TEST(SerialLink, EndpointSendProducesParsableFrameOnTheWire)
{
  const PtyPair pty;
  ASSERT_TRUE(pty.isReady());
  const SerialLink link(pty.slave(), baudRate);
  const mav::MavlinkEndpoint endpoint(link, MAVLINK_COMM_3);

  const mavlink_message_t heartbeat = mav::pack_heartbeat(mav::kVehicle, {.type = MAV_TYPE_QUADROTOR, .system_status = MAV_STATE_ACTIVE});
  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> expected{};
  const uint16_t length = mavlink_msg_to_send_buffer(expected.data(), &heartbeat);

  endpoint.send(heartbeat);

  const std::vector<uint8_t> onWire = pty.readFromLink(length);
  EXPECT_EQ(onWire, std::vector<uint8_t>(expected.begin(), expected.begin() + length));
}
