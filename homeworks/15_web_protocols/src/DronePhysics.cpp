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

  // Новий стан у локальних змінних, поза замком
  Coord newPos = CURRENT_POS;
  float newSpeed = CURRENT_SPEED;
  float newDir = CURRENT_DIR;
  const float newTime = timeSinceStart + deltaTime;

  switch (currentCommand.state) {
    case DroneState::Stopped:
      newDir = rotateToward(CURRENT_DIR, currentCommand.targetDir, config.angularSpeed, deltaTime);
      break;

    case DroneState::Turning:
      newDir = CURRENT_DIR + currentCommand.angleSpeed * deltaTime;
      break;

    case DroneState::Accelerating:
      newPos = movePos(CURRENT_POS, CURRENT_DIR, CURRENT_SPEED, deltaTime);
      newDir = rotateToward(CURRENT_DIR, currentCommand.targetDir, config.angularSpeed, deltaTime);
      newSpeed = fminf(CURRENT_SPEED + droneAcceleration * deltaTime, config.v0);
      break;

    case DroneState::Moving:
      newDir = rotateToward(CURRENT_DIR, currentCommand.targetDir, config.angularSpeed, deltaTime);
      newPos = movePos(CURRENT_POS, newDir, CURRENT_SPEED, deltaTime);
      break;

    case DroneState::Decelerating:
      newPos = movePos(CURRENT_POS, CURRENT_DIR, CURRENT_SPEED, deltaTime);
      newSpeed = fmaxf(CURRENT_SPEED - droneAcceleration * deltaTime, 0.0f);
      break;
  }

  {
    std::lock_guard<std::mutex> lock(stateMutex);
    CURRENT_POS = newPos;
    CURRENT_SPEED = newSpeed;
    CURRENT_DIR = newDir;
    timeSinceStart = newTime;
  }

  // Знімок фізики на кожній межі кратній simTimeStep, місія логуватиме саме їх,
  // тож dt у логах буде рівним. halfStep: timeSinceStart через похибку float
  // точно на межу не лягає, тому ловиться найближчий крок.
  const float halfStep = deltaTime * 0.5f;
  if (newTime >= nextSnapshotTime - halfStep) {
    snapshotQueue.push({
      .pos = newPos,
      .speed = newSpeed,
      .dir = newDir,
      .timeSinceStart = newTime,
    });
    nextSnapshotTime += config.simTimeStep;
  }
}

void DronePhysics::run()
{
  threadReady = true;
  LOG("DronePhysics thread ready");

  while (!startFlag) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  LOG("DronePhysics started");

  // Початковий знімок, стартова позиція дрона.
  snapshotQueue.push(getTelemetry());
  nextSnapshotTime = config.simTimeStep;

  auto last = std::chrono::steady_clock::now();

  // Модельний час який фізика має відкрокувати
  float accumulator = 0.0f;
  while (!stopFlag) {
    // Додавання до модельного часу фактично минулий час (реальний час x timeScale),
    // для захисту від неточних снів
    const auto now = std::chrono::steady_clock::now();
    accumulator += std::chrono::duration<float>(now - last).count() * config.timeScale;
    last = now;

    // Максимально допустимий модельний час, щоб у випадку чого не наздоганяти "лавиною кроків"
    const float maxCatchUp = 1.0f;
    if (accumulator > maxCatchUp) {
      accumulator = maxCatchUp;
    }

    // Крокуємо кроки фізики по заданому кроку фізики на весь модельний час який накопичився
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

DroneTelemetry DronePhysics::getTelemetry() const
{
  std::lock_guard<std::mutex> lock(stateMutex);

  return {
    .pos = CURRENT_POS,
    .speed = CURRENT_SPEED,
    .dir = CURRENT_DIR,
    .timeSinceStart = timeSinceStart,
  };
}

// Безпечно чекаємо актуального знімку фізики, бо поки міся чекає фізика точно робить знімки
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
