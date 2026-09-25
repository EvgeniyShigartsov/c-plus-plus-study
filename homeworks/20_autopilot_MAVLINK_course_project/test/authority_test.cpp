#include <gtest/gtest.h>

#include "authority/AuthorityStateMachine.hpp"

TEST(AuthorityStateMachineTest, StartsInStandby)
{
  const AuthorityStateMachine sm;
  EXPECT_EQ(sm.state(), AuthorityState::Standby);
}

TEST(AuthorityStateMachineTest, StandbyIgnoresEverythingExceptEnable)
{
  AuthorityStateMachine sm;

  sm.update({.enabled = false, .hasMission = true, .operatorInDeadband = false, .operatorHeartbeatOk = false});

  EXPECT_EQ(sm.state(), AuthorityState::Standby);
}

TEST(AuthorityStateMachineTest, EnableMovesStandbyToArmed)
{
  AuthorityStateMachine sm;

  sm.update({.enabled = true});

  EXPECT_EQ(sm.state(), AuthorityState::Armed);
}

TEST(AuthorityStateMachineTest, ArmedWaitsForMissionBeforeEngaging)
{
  AuthorityStateMachine sm;
  sm.update({.enabled = true});

  sm.update({.enabled = true, .hasMission = false});

  EXPECT_EQ(sm.state(), AuthorityState::Armed);
}

TEST(AuthorityStateMachineTest, MissionReadyMovesArmedToEngaged)
{
  AuthorityStateMachine sm;
  sm.update({.enabled = true});

  sm.update({.enabled = true, .hasMission = true});

  EXPECT_EQ(sm.state(), AuthorityState::Engaged);
}

TEST(AuthorityStateMachineTest, DisablingArmedReturnsToStandby)
{
  AuthorityStateMachine sm;
  sm.update({.enabled = true});

  sm.update({.enabled = false});

  EXPECT_EQ(sm.state(), AuthorityState::Standby);
}

TEST(AuthorityStateMachineTest, EngagedYieldsWhenOperatorLeavesDeadband)
{
  AuthorityStateMachine sm;
  sm.update({.enabled = true});
  sm.update({.enabled = true, .hasMission = true});

  sm.update({.enabled = true, .hasMission = true, .operatorInDeadband = false});

  EXPECT_EQ(sm.state(), AuthorityState::Yielding);
}

TEST(AuthorityStateMachineTest, YieldingReturnsToEngagedWhenOperatorReleasesSticks)
{
  AuthorityStateMachine sm;
  sm.update({.enabled = true});
  sm.update({.enabled = true, .hasMission = true});
  sm.update({.enabled = true, .hasMission = true, .operatorInDeadband = false});

  sm.update({.enabled = true, .hasMission = true, .operatorInDeadband = true});

  EXPECT_EQ(sm.state(), AuthorityState::Engaged);
}

TEST(AuthorityStateMachineTest, EngagedDropsToArmedWhenMissionLost)
{
  AuthorityStateMachine sm;
  sm.update({.enabled = true});
  sm.update({.enabled = true, .hasMission = true});

  sm.update({.enabled = true, .hasMission = false});

  EXPECT_EQ(sm.state(), AuthorityState::Armed);
}

TEST(AuthorityStateMachineTest, EngagedReturnsToStandbyWhenDisabled)
{
  AuthorityStateMachine sm;
  sm.update({.enabled = true});
  sm.update({.enabled = true, .hasMission = true});

  sm.update({.enabled = false, .hasMission = true});

  EXPECT_EQ(sm.state(), AuthorityState::Standby);
}

TEST(AuthorityStateMachineTest, LostOperatorLinkTriggersFailsafeFromEngaged)
{
  AuthorityStateMachine sm;
  sm.update({.enabled = true});
  sm.update({.enabled = true, .hasMission = true});

  sm.update({.enabled = true, .hasMission = true, .operatorHeartbeatOk = false});

  EXPECT_EQ(sm.state(), AuthorityState::Failsafe);
}

TEST(AuthorityStateMachineTest, LostOperatorLinkTriggersFailsafeFromYielding)
{
  AuthorityStateMachine sm;
  sm.update({.enabled = true});
  sm.update({.enabled = true, .hasMission = true});
  sm.update({.enabled = true, .hasMission = true, .operatorInDeadband = false});

  sm.update({.enabled = true, .hasMission = true, .operatorInDeadband = false, .operatorHeartbeatOk = false});

  EXPECT_EQ(sm.state(), AuthorityState::Failsafe);
}

TEST(AuthorityStateMachineTest, StandbyIgnoresLostOperatorLink)
{
  AuthorityStateMachine sm;

  sm.update({.enabled = false, .operatorHeartbeatOk = false});

  EXPECT_EQ(sm.state(), AuthorityState::Standby);
}

TEST(AuthorityStateMachineTest, ArmedIgnoresLostOperatorLink)
{
  AuthorityStateMachine sm;
  sm.update({.enabled = true});

  sm.update({.enabled = true, .hasMission = false, .operatorHeartbeatOk = false});

  EXPECT_EQ(sm.state(), AuthorityState::Armed);
}

TEST(AuthorityStateMachineTest, FailsafeResumesEngagedWhenLinkRestored)
{
  AuthorityStateMachine sm;
  sm.update({.enabled = true});
  sm.update({.enabled = true, .hasMission = true});
  sm.update({.enabled = true, .hasMission = true, .operatorHeartbeatOk = false});

  sm.update({.enabled = true, .hasMission = true, .operatorHeartbeatOk = true});

  EXPECT_EQ(sm.state(), AuthorityState::Engaged);
}

TEST(AuthorityStateMachineTest, ReachingFirePointCompletesFromEngaged)
{
  AuthorityStateMachine sm;
  sm.update({.enabled = true});
  sm.update({.enabled = true, .hasMission = true});

  sm.update({.enabled = true, .hasMission = true, .reachedFirePoint = true});

  EXPECT_EQ(sm.state(), AuthorityState::Complete);
}

TEST(AuthorityStateMachineTest, ReachingFirePointCompletesFromFailsafe)
{
  AuthorityStateMachine sm;
  sm.update({.enabled = true});
  sm.update({.enabled = true, .hasMission = true});
  sm.update({.enabled = true, .hasMission = true, .operatorHeartbeatOk = false});

  sm.update({.enabled = true, .hasMission = true, .operatorHeartbeatOk = false, .reachedFirePoint = true});

  EXPECT_EQ(sm.state(), AuthorityState::Complete);
}

TEST(AuthorityStateMachineTest, FirePointTakesPriorityOverDisableAndYield)
{
  AuthorityStateMachine sm;
  sm.update({.enabled = true});
  sm.update({.enabled = true, .hasMission = true});

  sm.update({.enabled = false, .hasMission = true, .operatorInDeadband = false, .reachedFirePoint = true});

  EXPECT_EQ(sm.state(), AuthorityState::Complete);
}

TEST(AuthorityStateMachineTest, CompleteIsTerminal)
{
  AuthorityStateMachine sm;
  sm.update({.enabled = true});
  sm.update({.enabled = true, .hasMission = true});
  sm.update({.enabled = true, .hasMission = true, .reachedFirePoint = true});

  sm.update({.enabled = true, .hasMission = true, .operatorInDeadband = false, .operatorHeartbeatOk = false});

  EXPECT_EQ(sm.state(), AuthorityState::Complete);
}

TEST(AuthorityStateMachineTest, UpdateReportsWhetherStateChanged)
{
  AuthorityStateMachine sm;

  EXPECT_TRUE(sm.update({.enabled = true}));
  EXPECT_FALSE(sm.update({.enabled = true}));
}

TEST(AuthorityStateMachineTest, HasControlOnlyInEngagedAndFailsafe)
{
  AuthorityStateMachine sm;
  EXPECT_FALSE(sm.hasControl());  // Standby

  sm.update({.enabled = true});
  EXPECT_FALSE(sm.hasControl());  // Armed

  sm.update({.enabled = true, .hasMission = true});
  EXPECT_TRUE(sm.hasControl());  // Engaged

  sm.update({.enabled = true, .hasMission = true, .operatorInDeadband = false});
  EXPECT_FALSE(sm.hasControl());  // Yielding

  sm.update({.enabled = true, .hasMission = true, .operatorInDeadband = false, .operatorHeartbeatOk = false});
  EXPECT_TRUE(sm.hasControl());  // Failsafe
}
