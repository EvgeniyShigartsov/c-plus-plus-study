#include <gtest/gtest.h>

#include "sim/DronePhysics.hpp"

namespace {

DroneConfig makeConfig()
{
  return {
    .startPos = {.x = 0.0f, .y = 0.0f},
    .altitude = 0.0f,
    .initialDir = 0.0f,
    .v0 = 10.0f,
    .accelerationPath = 5.0f,
    .simTimeStep = 0.0f,
    .hitRadius = 0.0f,
    .angularSpeed = 2.0f,
    .turnThreshold = 0.0f,
  };
}

}  // namespace

TEST(DronePhysicsTest, NeutralControlKeepsDroneStill)
{
  DronePhysics physics = DronePhysics(makeConfig());

  physics.stepPhysics(0.5f);
  physics.stepPhysics(0.5f);

  const DroneTelemetry telemetry = physics.getTelemetry();
  const float expected_speed = 0.0f;
  const float expected_dir = 0.0f;

  EXPECT_FLOAT_EQ(telemetry.speed, expected_speed);
  EXPECT_FLOAT_EQ(telemetry.dir, expected_dir);
  EXPECT_FLOAT_EQ(telemetry.pos.x, 0.0f);
  EXPECT_FLOAT_EQ(telemetry.pos.y, 0.0f);
}

TEST(DronePhysicsTest, FullThrottleAcceleratesByConfiguredRate)
{
  DronePhysics physics = DronePhysics(makeConfig());
  physics.setControl({.accel = 1.0f, .turnRate = 0.0f});

  const float delta_time = 0.1f;
  physics.stepPhysics(delta_time);

  const float max_acceleration = 10.0f;
  const float expected_speed = max_acceleration * delta_time;

  EXPECT_FLOAT_EQ(physics.getTelemetry().speed, expected_speed);
}

TEST(DronePhysicsTest, SpeedNeverExceedsConfiguredMax)
{
  DronePhysics physics = DronePhysics(makeConfig());
  physics.setControl({.accel = 1.0f, .turnRate = 0.0f});

  physics.stepPhysics(2.0f);

  const float expected_speed = 10.0f;
  EXPECT_FLOAT_EQ(physics.getTelemetry().speed, expected_speed);
}

TEST(DronePhysicsTest, FullBrakeNeverGoesNegative)
{
  DronePhysics physics = DronePhysics(makeConfig());

  physics.setControl({.accel = 1.0f, .turnRate = 0.0f});
  physics.stepPhysics(0.1f);

  physics.setControl({.accel = -1.0f, .turnRate = 0.0f});
  physics.stepPhysics(1.0f);

  const float expected_speed = 0.0f;
  EXPECT_FLOAT_EQ(physics.getTelemetry().speed, expected_speed);
}

TEST(DronePhysicsTest, TurnRateChangesDirectionByConfiguredRate)
{
  DronePhysics physics = DronePhysics(makeConfig());

  physics.setControl({.accel = 0.0f, .turnRate = -1.0f});

  const float delta_time = 0.5f;
  physics.stepPhysics(delta_time);

  const float angular_speed = 2.0f;
  const float expected_dir = -angular_speed * delta_time;

  EXPECT_FLOAT_EQ(physics.getTelemetry().dir, expected_dir);
}

TEST(DronePhysicsTest, PositionMovesAlongDirectionAtSpeed)
{
  DronePhysics physics = DronePhysics(makeConfig());

  physics.setControl({.accel = 1.0f, .turnRate = 0.0f});

  const float delta_time = 0.1f;
  physics.stepPhysics(delta_time);

  const float expected_x = 1.0f * delta_time;

  const DroneTelemetry telemetry = physics.getTelemetry();
  EXPECT_NEAR(telemetry.pos.x, expected_x, 1e-5f);
  EXPECT_NEAR(telemetry.pos.y, 0.0f, 1e-5f);
}
