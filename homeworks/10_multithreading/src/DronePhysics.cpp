#include "DronePhysics.hpp"
#include <cmath>
#include <chrono>
#include <mutex>
#include <thread>
#include "types.hpp"
#include "MathUtils.hpp"
#include "Logger.hpp"

DronePhysics::DronePhysics(const DroneConfig& config)
  : CURRENT_POS(config.startPos)
  , CURRENT_DIR(config.initialDir)
  , config(config)
  , droneAcceleration(powf(config.v0, 2) / (2 * config.accelerationPath))
{
}

void DronePhysics::pushCommand(const DroneCommand& cmd)
{
  commandQueue.push(cmd);
}

void DronePhysics::stepPhysics(const float deltaTime)
{
  const auto cmd = commandQueue.tryPop();

  if (cmd.has_value()) {
    currentCommand = cmd.value();
  }

  std::lock_guard<std::mutex> lock(stateMutex);
  switch (currentCommand.state) {
    case DroneState::Stopped:
      rotateTowards(currentCommand.targetDir, deltaTime);
      timeSinceStart += deltaTime;
      break;

    case DroneState::Turning:
      CURRENT_DIR += currentCommand.angleSpeed * deltaTime;
      timeSinceStart += deltaTime;
      break;

    case DroneState::Accelerating:
      updateDroneXY(deltaTime);

      rotateTowards(currentCommand.targetDir, deltaTime);
      CURRENT_SPEED += droneAcceleration * deltaTime;
      if (CURRENT_SPEED >= config.v0) {
        CURRENT_SPEED = config.v0;
      }
      timeSinceStart += deltaTime;
      break;

    case DroneState::Moving:
      rotateTowards(currentCommand.targetDir, deltaTime);
      updateDroneXY(deltaTime);
      timeSinceStart += deltaTime;
      break;

    case DroneState::Decelerating:
      updateDroneXY(deltaTime);

      CURRENT_SPEED -= droneAcceleration * deltaTime;
      if (CURRENT_SPEED <= 0.0f) {
        CURRENT_SPEED = 0.0f;
      }
      timeSinceStart += deltaTime;
      break;
  }

  if (timeSinceStart >= nextSnapshotTime - deltaTime * 0.5f) {
    snapshotQueue.push({
      .pos = CURRENT_POS,
      .speed = CURRENT_SPEED,
      .dir = CURRENT_DIR,
      .timeSinceStart = timeSinceStart,
    });
    nextSnapshotTime += config.simTimeStep;
  }
}

void DronePhysics::run()
{
  threadReady = true;
  LOG("DronePhysics thread ready");

  while (!startFlag) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));  // чекаємо start()
  }

  LOG("DronePhysics started");

  snapshotQueue.push(getTelemetry());
  nextSnapshotTime = config.simTimeStep;

  auto last = std::chrono::steady_clock::now();
  float accumulator = 0.0f;
  while (!stopFlag) {
    const auto now = std::chrono::steady_clock::now();
    accumulator += std::chrono::duration<float>(now - last).count() * config.timeScale;
    last = now;

    const float maxCatchUp = 1.0f;
    if (accumulator > maxCatchUp) {
      accumulator = maxCatchUp;
    }

    while (accumulator >= config.physicsTimeStep) {
      stepPhysics(config.physicsTimeStep);
      accumulator -= config.physicsTimeStep;
    }

    std::this_thread::sleep_for(std::chrono::duration<float>(config.physicsTimeStep / config.timeScale));
  }

  LOG("DronePhysics stopped");
}
void DronePhysics::start()
{
  startFlag = true;
}
void DronePhysics::stop()
{
  stopFlag = true;
}

bool DronePhysics::isThreadReady() const
{
  return threadReady;
}

DroneTelemetry DronePhysics::getTelemetry()
{
  std::lock_guard<std::mutex> lock(stateMutex);

  return {
    .pos = CURRENT_POS,
    .speed = CURRENT_SPEED,
    .dir = CURRENT_DIR,
    .timeSinceStart = timeSinceStart,
  };
}

DroneTelemetry DronePhysics::waitSnapshot()
{
  while (true) {
    const auto snapshot = snapshotQueue.tryPop();

    if (snapshot.has_value()) {
      return snapshot.value();
    }

    std::this_thread::sleep_for(std::chrono::duration<float>(config.physicsTimeStep / config.timeScale));
  }
}

void DronePhysics::updateDroneXY(const float deltaTime)
{
  CURRENT_POS = CURRENT_POS + Coord{cosf(CURRENT_DIR), sinf(CURRENT_DIR)} * CURRENT_SPEED * deltaTime;
}

// Плавна зміна курсу, не швидше за angularSpeed.
void DronePhysics::rotateTowards(const float targetDir, const float deltaTime)
{
  const float diff = normalizeAngle(targetDir - CURRENT_DIR);
  const float maxStep = config.angularSpeed * deltaTime;

  if (diff > maxStep) {
    CURRENT_DIR += maxStep;
  }
  else if (diff < -maxStep) {
    CURRENT_DIR -= maxStep;
  }
  else {
    CURRENT_DIR = targetDir;
  }
}