#pragma once
#include <cmath>
#include <cstring>
#include <string>

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
  std::string ammoName;
  Coord startPos;
  float altitude;
  float initialDir;
  float v0;
  float accelerationPath;
  float arrayTimeStep;
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
  Coord dropPoint;
  Coord aimPoint;
  Coord predictedTarget;
  float direction;
  std::string state;
  int targetIdx;
  int step;
};

struct Simulation {
  Coord CURRENT_POS = {0.0f, 0.0f};
  float CURRENT_TIME = 0.0f;
  float CURRENT_SPEED = 0.0f;
  float CURRENT_DIR = 0.0f;
  Coord maneuverPoint = {0.0f, 0.0f};
  float turningTimeLeft = 0.0f;

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

  Simulation() = default;
  // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
  Simulation(DroneConfig& droneConfig)
    : CURRENT_POS(droneConfig.startPos)
    , CURRENT_DIR(droneConfig.initialDir)
    , dc(droneConfig)
    , droneAcceleration(powf(dc.v0, 2) / (2 * dc.accelerationPath))  // (a)
    {};

  void updateDroneXY() { CURRENT_POS = CURRENT_POS + Coord{float(cosf(CURRENT_DIR)), sinf(CURRENT_DIR)} * CURRENT_SPEED * dc.simTimeStep; }
};

struct InterpolationIndex {
  float frac;
  int idx;
  int next;
};

struct Target {
  Coord pos;
  Coord velocity;
};