#pragma once
#include <atomic>
#include <mutex>
#include <thread>
#include "types.hpp"
#include "ThreadSafeQueue.hpp"

class DronePhysics {
public:
  DronePhysics(const DroneConfig& config);

  // Фаза 1
  void pushCommand(const DroneCommand& cmd);
  void stepPhysics(const float deltaTime);

  // Фаза 2
  void run();
  void start();
  void stop();
  void setThread(std::thread t);
  bool isThreadReady() const;

  DroneTelemetry getTelemetry();

private:
  Coord CURRENT_POS;
  float CURRENT_SPEED = 0.0f;
  float CURRENT_DIR = 0.0f;
  float timeSinceStart = 0.0f;

  DroneConfig config;
  float droneAcceleration = 0.0f;

  // Фаза 2
  ThreadSafeQueue<DroneCommand> commandQueue;
  DroneCommand currentCommand{};
  std::mutex stateMutex;
  std::atomic<bool> stopFlag{false};
  std::atomic<bool> startFlag{false};
  std::atomic<bool> threadReady{false};
  std::thread thread;

  void updateDroneXY(const float deltaTime);
};