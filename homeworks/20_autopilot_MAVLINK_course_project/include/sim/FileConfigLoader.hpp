#pragma once
#include "types.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class FileConfigLoader {
private:
  DroneConfig droneConfig{};
  BombParams bombParams{};
  std::string ammoName;
  float arrayTimeStep = 0.0f;
  float physicsTimeStep = 0.0f;
  float timeScale = 0.0f;

  bool readDroneConfig(const std::string& pathToConfig);
  bool readBombParams(const std::string& bombParamsPath);

public:
  bool load(const std::string& pathToConfig, const std::string& bombParamsPath);

  DroneConfig getConfig() const;
  BombParams getAmmoParams() const;
  float getArrayTimeStep() const;
  float getPhysicsTimeStep() const;
  float getTimeScale() const;

  virtual ~FileConfigLoader();
};
