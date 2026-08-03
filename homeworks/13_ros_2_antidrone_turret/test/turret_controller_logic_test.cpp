#include <gtest/gtest.h>

#include "antidrone_turret/actuator_model.hpp"
#include "antidrone_turret/turret_controller_logic.hpp"

namespace {

using antidrone_turret::ActionState;
using antidrone_turret::ActuatorState;
using antidrone_turret::GimbalDirection;
using antidrone_turret::kFrameCenterX;
using antidrone_turret::kFrameCenterY;
using antidrone_turret::ServoDirection;
using antidrone_turret::TargetInput;
using antidrone_turret::TargetState;
using antidrone_turret::TriggerState;
using antidrone_turret::TurretConfig;

TEST(TurretControllerLogicTest, LowConfidenceStaysIdleAndSkips)
{
  const TurretConfig config = {};
  const TargetInput target = {
    .visible = true,
    .x = 400.0f,
    .y = 200.0f,
    .distance_m = 20.0f,
    .confidence = 0.50f,
  };

  const auto decision = antidrone_turret::decide(target, ActuatorState::kReady, config);

  EXPECT_EQ(decision.status.target_state, TargetState::kLowConfidence);
  EXPECT_EQ(decision.status.action, ActionState::kIdle);
  EXPECT_EQ(decision.status.trigger_state, TriggerState::kSkip);
  EXPECT_FALSE(decision.should_aim);
  EXPECT_FALSE(decision.request_trigger);
}

TEST(TurretControllerLogicTest, InvisibleTargetIsNone)
{
  const TurretConfig config = {};
  const TargetInput target = {
    .visible = false,
    .x = 400.0f,
    .y = 200.0f,
    .distance_m = 10.0f,
    .confidence = 0.99f,
  };

  const auto decision = antidrone_turret::decide(target, ActuatorState::kReady, config);

  EXPECT_EQ(decision.status.target_state, TargetState::kNone);
  EXPECT_EQ(decision.status.action, ActionState::kIdle);
  EXPECT_EQ(decision.status.trigger_state, TriggerState::kSkip);
  EXPECT_FALSE(decision.should_aim);
}

TEST(TurretControllerLogicTest, ServoTurnsRightWhenTargetIsRightOfCenter)
{
  const auto servo = antidrone_turret::compute_servo_command(420.0f);

  EXPECT_EQ(servo.direction, ServoDirection::kRight);
  EXPECT_FLOAT_EQ(servo.target_x, 420.0f);
  EXPECT_FLOAT_EQ(servo.error_x, 100.0f);
  EXPECT_GT(servo.error_x, 0.0f);
}

TEST(TurretControllerLogicTest, ServoTurnsLeftAndCenters)
{
  const auto left = antidrone_turret::compute_servo_command(200.0f);
  EXPECT_EQ(left.direction, ServoDirection::kLeft);
  EXPECT_LT(left.error_x, 0.0f);

  const auto center = antidrone_turret::compute_servo_command(kFrameCenterX);
  EXPECT_EQ(center.direction, ServoDirection::kCenter);
  EXPECT_FLOAT_EQ(center.error_x, 0.0f);
}

TEST(TurretControllerLogicTest, GimbalGoesUpWhenTargetIsAboveCenter)
{
  const auto gimbal = antidrone_turret::compute_gimbal_command(180.0f);

  EXPECT_EQ(gimbal.direction, GimbalDirection::kUp);
  EXPECT_FLOAT_EQ(gimbal.target_y, 180.0f);
  EXPECT_FLOAT_EQ(gimbal.error_y, 60.0f);  // 240 - 180
  EXPECT_GT(gimbal.error_y, 0.0f);
}

TEST(TurretControllerLogicTest, GimbalGoesDownAndCenters)
{
  const auto down = antidrone_turret::compute_gimbal_command(300.0f);
  EXPECT_EQ(down.direction, GimbalDirection::kDown);
  EXPECT_LT(down.error_y, 0.0f);  // 240 - 300 = -60

  const auto center = antidrone_turret::compute_gimbal_command(kFrameCenterY);
  EXPECT_EQ(center.direction, GimbalDirection::kCenter);
  EXPECT_FLOAT_EQ(center.error_y, 0.0f);
}

TEST(TurretControllerLogicTest, CloseTargetWithReadyActuatorRequestsTrigger)
{
  const TurretConfig config = {};
  const TargetInput target = {
    .visible = true,
    .x = kFrameCenterX,
    .y = kFrameCenterY,
    .distance_m = 25.0f,
    .confidence = 0.90f,
  };

  const auto decision = antidrone_turret::decide(target, ActuatorState::kReady, config);

  EXPECT_EQ(decision.status.target_state, TargetState::kLocked);
  EXPECT_EQ(decision.status.action, ActionState::kTrack);
  EXPECT_EQ(decision.status.trigger_state, TriggerState::kRequested);
  EXPECT_TRUE(decision.should_aim);
  EXPECT_TRUE(decision.request_trigger);
}

TEST(TurretControllerLogicTest, CloseTargetWhileReloadingDoesNotRetrigger)
{
  const TurretConfig config = {};
  const TargetInput target = {
    .visible = true,
    .x = kFrameCenterX,
    .y = kFrameCenterY,
    .distance_m = 7.0f,
    .confidence = 0.92f,
  };

  const auto decision = antidrone_turret::decide(target, ActuatorState::kReloading, config);

  EXPECT_EQ(decision.status.target_state, TargetState::kLocked);
  EXPECT_EQ(decision.status.action, ActionState::kTrack);
  EXPECT_EQ(decision.status.trigger_state, TriggerState::kReloading);
  EXPECT_TRUE(decision.should_aim);
  EXPECT_FALSE(decision.request_trigger);
}

TEST(TurretControllerLogicTest, FarValidTargetLocksAndTracksButSkipsTrigger)
{
  const TurretConfig config = {};
  const TargetInput target = {
    .visible = true,
    .x = 420.0f,
    .y = 180.0f,
    .distance_m = 55.0f,
    .confidence = 0.85f,
  };

  const auto decision = antidrone_turret::decide(target, ActuatorState::kReady, config);

  EXPECT_EQ(decision.status.target_state, TargetState::kLocked);
  EXPECT_EQ(decision.status.action, ActionState::kTrack);
  EXPECT_EQ(decision.status.trigger_state, TriggerState::kSkip);
  EXPECT_TRUE(decision.should_aim);
  EXPECT_FALSE(decision.request_trigger);

  EXPECT_FLOAT_EQ(decision.status.confidence, target.confidence);
  EXPECT_FLOAT_EQ(decision.status.distance_m, target.distance_m);
}

TEST(TurretControllerLogicTest, ConfidenceExactlyAtThresholdLocks)
{
  const TurretConfig config = {};
  const TargetInput target = {
    .visible = true,
    .x = kFrameCenterX,
    .y = kFrameCenterY,
    .distance_m = 100.0f,
    .confidence = 0.80f,
  };

  const auto state = antidrone_turret::evaluate_target(target, config);

  EXPECT_EQ(state, TargetState::kLocked);
}
}  // namespace
