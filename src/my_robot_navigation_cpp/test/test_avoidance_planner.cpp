// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Anson Antony

// Unit tests for the C++ planner. They mirror test_avoidance_planner.py so both
// implementations are held to the same behaviour.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

#include "my_robot_navigation_cpp/avoidance_planner.hpp"
#include "my_robot_navigation_cpp/waypoints.hpp"

namespace nav = my_robot_navigation_cpp;
using nav::NavigationState;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kInc = 2.0 * kPi / 360.0;

std::vector<double> scan360(double value = 3.0) {return std::vector<double>(360, value);}

nav::AvoidanceDecision plan(
  nav::AvoidancePlanner & p, const std::vector<double> & ranges,
  std::optional<double> goal_distance = std::nullopt, double goal_angle = 0.0,
  bool paused = false)
{
  return p.plan(nav::ScanInput{ranges, 0.0, kInc, 0.05, 3.5}, paused, goal_distance, goal_angle);
}

nav::PlannerConfig config(const char * algorithm)
{
  nav::PlannerConfig c;
  c.algorithm = algorithm;
  return c;
}

}  // namespace

TEST(SanitizeScan, ReplacesInvalidReadingsWithRangeMax) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const auto cleaned = nav::sanitize_scan({nan, inf, -1.0, 0.0, 0.01, 0.5, 5.0}, 0.05, 3.5);
  const std::vector<double> expected{3.5, 3.5, 3.5, 3.5, 3.5, 0.5, 3.5};
  EXPECT_EQ(cleaned, expected);
}

TEST(SectorPlanner, ClearPathDrivesAtMaxSpeed) {
  nav::AvoidancePlanner p(config("sector"));
  const auto d = plan(p, scan360());
  EXPECT_EQ(d.state, NavigationState::kClear);
  EXPECT_NEAR(d.linear_x, 0.22, 1e-9);
  EXPECT_LT(std::abs(d.angular_z), 0.05);
}

TEST(SectorPlanner, ObstacleOnRightTurnsLeft) {
  nav::AvoidancePlanner p(config("sector"));
  auto s = scan360();
  for (int i = 315; i < 345; ++i) {
    s[i] = 0.45;
  }
  const auto d = plan(p, s);
  EXPECT_EQ(d.state, NavigationState::kAvoiding);
  EXPECT_GT(d.angular_z, 0.0);
  EXPECT_GT(d.linear_x, 0.05);
  EXPECT_LT(d.linear_x, 0.22);
}

TEST(SectorPlanner, ObstacleOnLeftTurnsRight) {
  nav::AvoidancePlanner p(config("sector"));
  auto s = scan360();
  for (int i = 15; i < 45; ++i) {
    s[i] = 0.45;
  }
  EXPECT_LT(plan(p, s).angular_z, 0.0);
}

TEST(SectorPlanner, CriticalObstacleStopsForwardMotion) {
  nav::AvoidancePlanner p(config("sector"));
  auto s = scan360();
  s[0] = 0.20;
  const auto d = plan(p, s);
  EXPECT_EQ(d.state, NavigationState::kCriticalStop);
  EXPECT_EQ(d.linear_x, 0.0);
  EXPECT_NEAR(std::abs(d.angular_z), 1.2, 1e-9);
}

TEST(SectorPlanner, TurnHysteresisKeepsDirectionOnSmallChanges) {
  nav::AvoidancePlanner p(config("sector"));
  auto s1 = scan360();
  s1[0] = 0.45;
  s1[30] = 0.90;
  s1[330] = 0.70;
  EXPECT_GT(plan(p, s1).angular_z, 0.0);
  auto s2 = scan360();
  s2[0] = 0.45;
  s2[30] = 0.79;
  s2[330] = 0.81;  // 2 cm in favour of the right: inside the 8 cm dead band
  EXPECT_GT(plan(p, s2).angular_z, 0.0);
}

TEST(Recovery, TriggersAfterTimeoutAndReverses) {
  auto c = config("sector");
  c.stuck_timeout_steps = 3;
  nav::AvoidancePlanner p(c);
  auto s = scan360();
  s[0] = 0.18;
  plan(p, s);
  plan(p, s);
  const auto d = plan(p, s);
  EXPECT_EQ(d.state, NavigationState::kRecovery);
  EXPECT_LT(d.linear_x, 0.0);
}

TEST(Recovery, DoesNotReverseIntoRearObstacle) {
  auto c = config("sector");
  c.stuck_timeout_steps = 2;
  c.recovery_hold_steps = 4;
  nav::AvoidancePlanner p(c);
  auto s = scan360();
  s[0] = 0.18;
  s[180] = 0.18;
  plan(p, s);
  const auto d = plan(p, s);
  EXPECT_EQ(d.state, NavigationState::kRecovery);
  EXPECT_EQ(d.linear_x, 0.0);
  EXPECT_NE(d.angular_z, 0.0);
}

TEST(Planner, PausedOutputsZeroVelocity) {
  nav::AvoidancePlanner p;
  const auto d = plan(p, scan360(0.25), std::nullopt, 0.0, true);
  EXPECT_EQ(d.state, NavigationState::kPaused);
  EXPECT_EQ(d.linear_x, 0.0);
  EXPECT_EQ(d.angular_z, 0.0);
}

TEST(Planner, GoalReachedWithinTolerance) {
  nav::AvoidancePlanner p(config("apf"));
  const auto moving = plan(p, scan360(), 1.5, kPi / 4);
  EXPECT_EQ(moving.state, NavigationState::kClear);
  EXPECT_GT(moving.angular_z, 0.0);
  const auto arrived = plan(p, scan360(), 0.15, 0.1);
  EXPECT_EQ(arrived.state, NavigationState::kGoalReached);
  EXPECT_EQ(arrived.linear_x, 0.0);
}

TEST(Planner, UTrapTriggersWallFollow) {
  auto c = config("apf");
  c.trap_timeout_steps = 3;
  c.wall_follow_hold_steps = 4;
  nav::AvoidancePlanner p(c);
  auto s = scan360();
  for (int i = 320; i < 360; ++i) {
    s[i] = 0.45;
  }
  for (int i = 0; i < 41; ++i) {
    s[i] = 0.45;
  }
  nav::AvoidanceDecision d;
  for (int k = 0; k < 4; ++k) {
    d = plan(p, s, 2.0, 0.0);
  }
  EXPECT_EQ(d.state, NavigationState::kWallFollow);
  EXPECT_GT(d.linear_x, 0.0);
}

TEST(ApfPlanner, RepelsFromFrontRightObstacle) {
  nav::AvoidancePlanner p(config("apf"));
  auto s = scan360();
  for (int i = 320; i < 350; ++i) {
    s[i] = 0.45;
  }
  const auto d = plan(p, s);
  EXPECT_EQ(d.state, NavigationState::kAvoiding);
  EXPECT_GT(d.angular_z, 0.0);
}

TEST(ApfPlanner, ReachesGoalInFrontOfWall) {
  nav::AvoidancePlanner p(config("apf"));
  auto s = scan360();
  for (int i = 320; i < 360; ++i) {
    s[i] = 0.70;
  }
  for (int i = 0; i < 41; ++i) {
    s[i] = 0.70;
  }
  const auto d = plan(p, s, 0.40, 0.0);
  EXPECT_GT(d.linear_x, 0.0);
  EXPECT_LT(std::abs(d.angular_z), 0.1);
}

TEST(FollowTheGap, SteersTowardOpenSide) {
  nav::AvoidancePlanner p(config("follow_the_gap"));
  auto s = scan360();
  for (int i = 300; i < 360; ++i) {
    s[i] = 0.45;
  }
  for (int i = 0; i < 11; ++i) {
    s[i] = 0.45;
  }
  const auto d = plan(p, s);
  EXPECT_EQ(d.state, NavigationState::kAvoiding);
  EXPECT_GT(d.target_heading_rad, 0.0);
}

TEST(FollowTheGap, PrefersGapTowardGoal) {
  nav::AvoidancePlanner p(config("follow_the_gap"));
  auto s = scan360(0.50);
  for (int i = 5; i < 66; ++i) {
    s[i] = 3.0;
  }
  for (int i = 305; i < 351; ++i) {
    s[i] = 3.0;
  }
  const auto d = plan(p, s, 3.0, -30.0 * kPi / 180.0);
  EXPECT_LT(d.target_heading_rad, 0.0);
  EXPECT_LT(d.angular_z, 0.0);
}

TEST(Waypoints, ParsesAndRejectsBadInput) {
  EXPECT_TRUE(nav::parse_waypoints("").empty());
  const auto wps = nav::parse_waypoints("1,2; -0.5, 3.25;");
  ASSERT_EQ(wps.size(), 2u);
  EXPECT_DOUBLE_EQ(wps[1].first, -0.5);
  EXPECT_DOUBLE_EQ(wps[1].second, 3.25);
  EXPECT_THROW(nav::parse_waypoints("1,2,3"), std::invalid_argument);
  EXPECT_THROW(nav::parse_waypoints("1,abc"), std::invalid_argument);
}
