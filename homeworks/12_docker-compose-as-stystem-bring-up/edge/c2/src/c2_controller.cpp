#include "c2_controller.hpp"
#include "fc_link.hpp"     // MAVSDK обгортка, API описано у fc_link.hpp
#include "udp_socket.hpp"  // UDP прийом, API описано у udp_socket.hpp

#include <nlohmann/json.hpp>  // Розбiр JSON з точками маршруту вiд auto_stub

#include <array>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

using json = nlohmann::json;

static constexpr uint16_t STUB_PORT = 14560;
const std::string LOG_FILE_PATH = "/var/log/c2/c2.log";
const std::string HEALTHCHECK_FILE_PATH = "/tmp/c2_healthy";

namespace {

std::string getStateName(C2State name)
{
  switch (name) {
    case C2State::DISARMED:
      return "DISARMED";
    case C2State::ARMED_HOLD:
      return "ARMED_HOLD";
    case C2State::ARMED_GUIDED:
      return "ARMED_GUIDED";
    case C2State::ARMED_MANUAL:
      return "ARMED_MANUAL";
    default:
      return "UNKNOWN";
  }
}
}  // namespace

struct C2Controller::Impl {
  C2State state = C2State::DISARMED;

  FcLink flightControl;
  UdpSocket stub_sock = UdpSocket(STUB_PORT);
  std::ofstream log;
  bool healthy_reported = false;

  explicit Impl(uint16_t fc_port)
    : flightControl(fc_port)
    , log(LOG_FILE_PATH, std::ios::app)
  {
  }

  void logLine(const std::string& line)
  {
    std::cout << line << std::endl;
    if (log.is_open()) {
      log << line << std::endl;
    }
  }

  void makeTransition(C2State nextState)
  {
    if (nextState == state) {
      return;
    }
    logLine(std::string("[C2] state: ") + getStateName(state) + " -> " + getStateName(nextState));
    state = nextState;
  }
};

C2Controller::C2Controller(uint16_t fc_port)
  : impl_(std::make_unique<Impl>(fc_port))
{
}

C2Controller::~C2Controller() = default;

void C2Controller::tick()
{
  Impl& impl = *impl_;

  if (!impl.healthy_reported && impl.flightControl.is_connected()) {
    std::ofstream(HEALTHCHECK_FILE_PATH).close();
    impl.healthy_reported = true;
  }

  C2State next = C2State::DISARMED;
  if (impl.flightControl.is_armed()) {
    switch (impl.flightControl.flight_mode()) {
      case FcLink::FlightMode::Guided:
        next = C2State::ARMED_GUIDED;
        break;
      case FcLink::FlightMode::Hold:
        next = C2State::ARMED_HOLD;
        break;
      case FcLink::FlightMode::Manual:
      case FcLink::FlightMode::Unknown:
        next = C2State::ARMED_MANUAL;
        break;
    }
  }

  const bool entered_hold = next == C2State::ARMED_HOLD && impl.state != C2State::ARMED_HOLD;
  impl.makeTransition(next);
  if (entered_hold) {
    impl.flightControl.hold();
  }

  // Точка маршруту вiд auto_stub; recv неблокуючий, -1 = даних немає.
  std::array<char, 1024> buf{};
  const ssize_t bytes = impl.stub_sock.recv(buf.data(), buf.size());
  if (bytes <= 0) {
    return;
  }

  if (impl.state != C2State::ARMED_GUIDED) {
    impl.logLine(std::string("[C2] blocked: waypoint in ") + getStateName(impl.state));
    return;
  }

  const std::string message(buf.data(), static_cast<std::size_t>(bytes));

  try {
    const auto waypoint = json::parse(message);
    const float north = waypoint.at("north_m").get<float>();
    const float east = waypoint.at("east_m").get<float>();

    impl.flightControl.go_to_ned(north, east);

    std::ostringstream oss;
    oss << "[C2] fwd: north=" << north << " east=" << east;
    impl.logLine(oss.str());
  }
  catch (const json::exception& error) {
    impl.logLine(std::string("[C2] error: invalid waypoint json: ") + error.what());
  }
}

C2State C2Controller::current_state() const
{
  return impl_->state;
}
