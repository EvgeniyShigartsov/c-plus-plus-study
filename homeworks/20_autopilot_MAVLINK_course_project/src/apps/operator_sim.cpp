// operator_sim — тестовий скриптований оператор
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "Logger.hpp"
#include "link/UdpLink.hpp"
#include "mavlink/Codec.hpp"
#include "mavlink/Endpoint.hpp"
#include "mavlink/RadioControl.hpp"
#include "sim/FileConfigLoader.hpp"
#include "sim/JsonTargetProvider.hpp"

#define OPERATOR_LOG(msg) LOG("[OPERATOR:] " << msg)
#define OPERATOR_DEBUG(msg) DEBUG("[OPERATOR:] " << msg)

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)

const std::string defaultDataDir = "homeworks/20_autopilot_MAVLINK_course_project/data";

struct CliOptions {
  std::string apHost = "127.0.0.1";
  uint16_t apPort = 14560;  // домашній порт autopilot, сюди шлемо HEARTBEAT/команди
  std::string vehicleHost = "127.0.0.1";
  uint16_t vehiclePort = 14555;  // домашній порт vehicle_sim, сюди шлемо команди від реального оператора
  std::string gcsHost = "127.0.0.1";
  uint16_t gcsPort = 14550;  // GCS-порт, куди vehicle_sim дублює телеметрію - звідси беремо реальний місійний час
  std::string configPath = defaultDataDir + "/config.json";
  std::string ammoPath = defaultDataDir + "/ammo.json";
  std::string targetsPath = defaultDataDir + "/targets.json";
  std::string operatorScenario;  // timeline-файл дій оператора
  float timeScale = 1.0f;        // аналогічно vehicle_sim, щоб час сценарію збігався з місією
};

CliOptions parseArgs(const std::vector<std::string>& args)
{
  CliOptions opts;

  for (size_t i = 0; i + 1 < args.size(); i += 2) {
    const std::string& key = args[i];
    const std::string& value = args[i + 1];

    if (key == "--ap-host") {
      opts.apHost = value;
    }
    else if (key == "--ap-port") {
      opts.apPort = static_cast<uint16_t>(std::stoi(value));
    }
    else if (key == "--vehicle-host") {
      opts.vehicleHost = value;
    }
    else if (key == "--vehicle-port") {
      opts.vehiclePort = static_cast<uint16_t>(std::stoi(value));
    }
    else if (key == "--gcs-host") {
      opts.gcsHost = value;
    }
    else if (key == "--gcs-port") {
      opts.gcsPort = static_cast<uint16_t>(std::stoi(value));
    }
    else if (key == "--operator-scenario") {
      opts.operatorScenario = value;
    }
    else if (key == "--config-path") {
      opts.configPath = value;
    }
    else if (key == "--ammo-path") {
      opts.ammoPath = value;
    }
    else if (key == "--targets") {
      opts.targetsPath = value;
    }
    else if (key == "--time-scale") {
      opts.timeScale = std::stof(value);
    }
    else {
      std::cerr << "Unknown argument at operator_sim.cpp: " << key << '\n';
    }
  }

  return opts;
}

// Один рядок timeline-файлу сценарію оператора:
// "<time> <action> [args...]", "#" на початку - коментар, порожні рядки пропускаються.
struct TimelineEvent {
  float time = 0.0f;
  std::string action;
  std::vector<std::string> args;
};

std::vector<TimelineEvent> loadOperatorScenario(const std::string& path)
{
  std::vector<TimelineEvent> events;
  std::ifstream file(path);
  if (!file) {
    OPERATOR_LOG("Failed to open operator scenario " << path);
    return events;
  }

  std::string line;
  while (std::getline(file, line)) {
    std::istringstream lineStream(line);
    std::string first;
    if (!(lineStream >> first) || first.starts_with('#')) {
      continue;  // порожній рядок або коментар
    }

    TimelineEvent event;
    event.time = std::stof(first);
    lineStream >> event.action;

    std::string arg;
    while (lineStream >> arg) {
      event.args.push_back(arg);
    }

    events.push_back(event);
  }

  return events;
}

struct OperatorChannelState {
  uint16_t roll = mav::kPwmNeutral;
  uint16_t throttle = mav::kPwmNeutral;
  float rollClearAt = -1.0f;      // у секундах
  float throttleClearAt = -1.0f;  // у секундах
};

void executeEvent(const TimelineEvent& event,
                  const mav::MavlinkEndpoint& endpoint,
                  OperatorChannelState& out_channelState,
                  float& out_heartbeatDropUntil,
                  bool& out_targetsArmed)
{
  if (event.action == "heartbeat_drop") {
    if (event.args.empty()) {
      OPERATOR_LOG("event: heartbeat_drop requires 1 arg (duration), skip");
      return;
    }

    const float duration = std::stof(event.args[0]);
    out_heartbeatDropUntil = event.time + duration;
    OPERATOR_LOG("event: heartbeat_drop for " << duration << "sec");
  }
  else if (event.action == "channel") {
    if (event.args.size() < 3) {
      OPERATOR_LOG("event: channel requires 3 args (channel offset duration), skip");
      return;
    }

    const std::string& channel = event.args[0];
    const int offset = std::stoi(event.args[1]);
    const float duration = std::stof(event.args[2]);
    const uint16_t pwm = static_cast<uint16_t>(mav::kPwmNeutral + offset);

    if (channel == "roll") {
      out_channelState.roll = pwm;
      out_channelState.rollClearAt = event.time + duration;
    }
    else if (channel == "throttle") {
      out_channelState.throttle = pwm;
      out_channelState.throttleClearAt = event.time + duration;
    }
    else {
      OPERATOR_LOG("event: unknown channel '" << channel << "'");
      return;
    }

    OPERATOR_LOG("event: channel " << channel << " = " << pwm << " on " << duration << "sec");
  }
  else if (event.action == "enable") {
    endpoint.send(mav::pack_enable_command(mav::kGcs, mav::kAutopilot, {.enabled = true}));
    OPERATOR_LOG("event: enable");
  }
  else if (event.action == "disable") {
    endpoint.send(mav::pack_enable_command(mav::kGcs, mav::kAutopilot, {.enabled = false}));
    OPERATOR_LOG("event: disable");
  }
  else if (event.action == "designate_targets") {
    out_targetsArmed = true;
    OPERATOR_LOG("event: designate_targets armed");
  }
  else {
    OPERATOR_LOG("event: unknown action '" << event.action << "', skip");
  }
}

int main(int argc, char* argv[])
{
  std::vector<std::string> args;
  for (int i = 1; i < argc; i++) {
    args.emplace_back(argv[i]);
  }
  const CliOptions opts = parseArgs(args);

  const bool isScenarioNotPesented = opts.operatorScenario.empty();

  OPERATOR_LOG("operator_sim:\n"
               << "  ap-host           = " << opts.apHost << '\n'
               << "  ap-port           = " << opts.apPort << '\n'
               << "  vehicle-host      = " << opts.vehicleHost << '\n'
               << "  vehicle-port      = " << opts.vehiclePort << '\n'
               << "  gcs-host          = " << opts.gcsHost << '\n'
               << "  gcs-port          = " << opts.gcsPort << '\n'
               << "  operator-scenario = " << (isScenarioNotPesented ? "(not presented)" : opts.operatorScenario) << '\n'
               << "  config-path       = " << opts.configPath << '\n'
               << "  ammo-path         = " << opts.ammoPath << '\n'
               << "  targets           = " << opts.targetsPath << '\n'
               << "  time-scale        = " << opts.timeScale);

  if (isScenarioNotPesented) {
    OPERATOR_LOG("operator-scenario should be presented & have valid markup");
    return 1;
  }

  FileConfigLoader loader;
  if (!loader.load(opts.configPath, opts.ammoPath)) {
    OPERATOR_LOG("Failed to load config or ammo");
    return 1;
  }

  const JsonTargetProvider targetProvider = JsonTargetProvider(opts.targetsPath, loader.getArrayTimeStep(), loader.getConfig().simTimeStep);

  if (!targetProvider.isLoadSucces()) {
    OPERATOR_LOG("Failed to load targets " << opts.targetsPath);
    return 1;
  }

  const UdpLink udp(opts.apHost, opts.apPort);
  if (!udp.isOpen()) {
    OPERATOR_LOG("Failed to open UDP link to " << opts.apHost << ":" << opts.apPort);
    return 1;
  }

  const std::vector<TimelineEvent> events = loadOperatorScenario(opts.operatorScenario);
  if (events.size() == 0) {
    OPERATOR_LOG("No events found");
    return 1;
  }

  const mav::MavlinkEndpoint endpoint(udp, MAVLINK_COMM_1);

  const UdpLink vehicleUdp(opts.vehicleHost, opts.vehiclePort);
  if (!vehicleUdp.isOpen()) {
    OPERATOR_LOG("Failed to open UDP link to " << opts.vehicleHost << ":" << opts.vehiclePort);
    return 1;
  }
  const mav::MavlinkEndpoint vehicleEndpoint(vehicleUdp, MAVLINK_COMM_2);

  const UdpLink gcsUdp(opts.gcsHost, opts.gcsPort, opts.gcsPort);
  if (!gcsUdp.isOpen()) {
    OPERATOR_LOG("Failed to open UDP link to " << opts.gcsHost << ":" << opts.gcsPort);
    return 1;
  }
  const mav::MavlinkEndpoint gcsEndpoint(gcsUdp, MAVLINK_COMM_3);

  const std::chrono::milliseconds kHeartbeatPeriod = std::chrono::milliseconds(500);  // 2 Гц
  std::chrono::steady_clock::time_point lastHeartbeat;

  std::chrono::steady_clock::time_point last = std::chrono::steady_clock::now();
  float scenarioTime = 0.0f;
  size_t nextEventIndex = 0;
  OperatorChannelState channelState;
  float heartbeatDropUntil = -1.0f;
  bool targetsArmed = false;
  float missionTime = 0.0f;  // реальний час дрона, з його телеметрії
  bool haveMissionTime = false;

  while (true) {
    const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

    scenarioTime += std::chrono::duration<float>(now - last).count() * opts.timeScale;
    last = now;

    while (nextEventIndex < events.size() && scenarioTime >= events[nextEventIndex].time) {
      executeEvent(events[nextEventIndex], endpoint, channelState, heartbeatDropUntil, targetsArmed);
      nextEventIndex++;
    }

    for (const mavlink_message_t& msg : gcsEndpoint.poll()) {
      if (msg.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
        const bool wasMissing = !haveMissionTime;
        missionTime = static_cast<float>(mav::parse_local_position_ned(msg).time_boot_ms) / 1000.0f;
        haveMissionTime = true;

        if (wasMissing) {
          OPERATOR_LOG("mission time: synced from vehicle telemetry, t=" << missionTime);
        }
      }
    }

    if (channelState.rollClearAt >= 0.0f && scenarioTime >= channelState.rollClearAt) {
      channelState.roll = mav::kPwmNeutral;
      channelState.rollClearAt = -1.0f;
    }
    if (channelState.throttleClearAt >= 0.0f && scenarioTime >= channelState.throttleClearAt) {
      channelState.throttle = mav::kPwmNeutral;
      channelState.throttleClearAt = -1.0f;
    }

    const bool heartbeatDropped = scenarioTime < heartbeatDropUntil;

    if (!heartbeatDropped && now - lastHeartbeat >= kHeartbeatPeriod) {
      endpoint.send(mav::pack_heartbeat(mav::kGcs, {.type = MAV_TYPE_GCS}));
      lastHeartbeat = now;
    }

    if (targetsArmed && haveMissionTime) {
      const int targetCount = targetProvider.getTargetCount();
      for (int i = 0; i < targetCount; i++) {
        const Coord pos = targetProvider.getTarget(missionTime, i).pos;
        endpoint.send(mav::pack_target_designation(mav::kGcs,
                                                   mav::kAutopilot,
                                                   {
                                                     .target_id = static_cast<uint8_t>(i),
                                                     .target_count = static_cast<uint8_t>(targetCount),
                                                     .latitude = pos.x,
                                                     .longitude = pos.y,
                                                   }));
      }
    }

    vehicleEndpoint.send(mav::pack_radio_control_channels_override(
      mav::kGcs, mav::kVehicle, mav::to_control_signal({.roll = channelState.roll, .throttle = channelState.throttle})));

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
