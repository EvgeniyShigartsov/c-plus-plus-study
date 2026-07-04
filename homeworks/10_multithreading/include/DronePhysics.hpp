#pragma once
#include <atomic>
#include <mutex>
#include "types.hpp"
#include "ThreadSafeQueue.hpp"

class DronePhysics {
public:
  DronePhysics(const DroneConfig& config);

  void pushCommand(const DroneCommand& cmd);
  void stepPhysics(const float deltaTime);

  void run();
  void start();
  void stop();
  bool isThreadReady() const;

  DroneTelemetry getTelemetry();
  DroneTelemetry waitSnapshot();

private:
  Coord CURRENT_POS;
  float CURRENT_SPEED = 0.0f;
  float CURRENT_DIR = 0.0f;
  float timeSinceStart = 0.0f;

  DroneConfig config;
  float droneAcceleration = 0.0f;

  ThreadSafeQueue<DroneCommand> commandQueue;
  ThreadSafeQueue<DroneTelemetry> snapshotQueue;
  float nextSnapshotTime = 0.0f;
  DroneCommand currentCommand{};
  std::mutex stateMutex;
  std::atomic<bool> stopFlag{false};
  std::atomic<bool> startFlag{false};
  std::atomic<bool> threadReady{false};

  void updateDroneXY(const float deltaTime);
  void rotateTowards(const float targetDir, const float deltaTime);
};