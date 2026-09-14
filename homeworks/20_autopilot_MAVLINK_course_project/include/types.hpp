#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

// Конвенція осей: x = схід, y = північ, dir = математичний курс, проти годинникової стрілки, старт зі сходу.
struct VehicleState {
  uint32_t mission_time_ms = 0;  // час від старту місії у мілісекундах
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;      // висота, метри
  float vx = 0.0f;     // швидкість по осі X, м/с
  float vy = 0.0f;     // швидкість по осі Y, м/с
  float speed = 0.0f;  // горизонтальна швидкість
  float dir = 0.0f;
  uint8_t state = 0;  //  стан стейт-машини донаведення (0..4) TODO: замінити на enum коли буде додано.
};

struct Coord {
  float x;
  float y;

  Coord operator+(const Coord& other) const
  {
    Coord result{x + other.x, y + other.y};
    return result;
  }

  Coord operator-(const Coord& other) const
  {
    Coord result{x - other.x, y - other.y};
    return result;
  }
  Coord operator*(const float scalar) const
  {
    Coord result{x * scalar, y * scalar};
    return result;
  }
  Coord operator/(const float divider) const
  {
    Coord result{x / divider, y / divider};
    return result;
  }
  bool operator==(const Coord& other) const { return fabsf(x - other.x) < 1e-6f && fabsf(y - other.y) < 1e-6f; }
};

struct DroneConfig {
  Coord startPos;
  float altitude;
  float initialDir;
  float v0;
  float accelerationPath;
  float simTimeStep;
  float hitRadius;
  float angularSpeed;
  float turnThreshold;
};

struct BombParams {
  std::string name;
  float mass = 0.0f;
  float drag = 0.0f;
  float lift = 0.0f;
};

struct SimStep {
  Coord pos;
  Coord dropPoint = {0.0f, 0.0f};
  Coord aimPoint = {0.0f, 0.0f};
  Coord predictedTarget = {0.0f, 0.0f};
  float direction = 0.0f;
  std::string state;
  int targetIdx;
  int step = 0;
  float timeSecSinceStart = 0.0f;
};

struct Simulation {
  Coord CURRENT_POS = {0.0f, 0.0f};
  float CURRENT_SPEED = 0.0f;
  float CURRENT_DIR = 0.0f;
  Coord maneuverPoint = {0.0f, 0.0f};

  int selectedTargetIndex = 0;
  int prevSelectedTargetIndex = 0;
  int step = 0;
  bool reachedFirePoint = false;
  bool needsManeuver = false;
  bool reachedManeuverPoint = false;

  // new fields
  float deltaAngle = 0.0f;
  float dirToFire = 0.0f;
  DroneConfig dc = {};
  float droneAcceleration = 0.0f;
  float timeSecSinceStart = 0.0f;

  Simulation() = default;
  Simulation(DroneConfig& droneConfig)
    : CURRENT_POS(droneConfig.startPos)
    , CURRENT_DIR(droneConfig.initialDir)
    , dc(droneConfig)
    , droneAcceleration(powf(dc.v0, 2) / (2 * dc.accelerationPath))  // (a)
    {};
};

struct Target {
  Coord pos;
  Coord velocity;
};

enum DroneState { Stopped, Turning, Accelerating, Moving, Decelerating };

struct DroneCommand {
  DroneState state;
  float targetDir;
};

struct DroneTelemetry {
  Coord pos;
  float speed;
  float dir;
  float timeSinceStart;
};

struct ControlSignal {
  float accel = 0.0f;     // прискорення вздовж курсу, [-1..1] (1 = повний газ, -1 = гальмо)
  float turnRate = 0.0f;  // швидкість повороту, [-1..1] (1 = макс. вліво, -1 = вправо)
};

struct TargetSlot {
  Target target = {{0.0f, 0.0f}, {0.0f, 0.0f}};
  float lastUpdateTime = 0.0f;
  bool seen = false;
};