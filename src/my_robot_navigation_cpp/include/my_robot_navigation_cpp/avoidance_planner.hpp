// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Anson Antony

// Obstacle avoidance planners (sector, APF, follow-the-gap). No ROS dependency:
// the same class is used by the ROS node, the unit tests and the Python binding.
// This is a port of my_robot_navigation/avoidance_planner.py and is kept
// numerically identical to it (checked by test/test_parity.py).

#ifndef MY_ROBOT_NAVIGATION_CPP__AVOIDANCE_PLANNER_HPP_
#define MY_ROBOT_NAVIGATION_CPP__AVOIDANCE_PLANNER_HPP_

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace my_robot_navigation_cpp
{

enum class NavigationState
{
  kPaused,
  kClear,
  kAvoiding,
  kCriticalStop,
  kRecovery,
  kWallFollow,
  kGoalReached,
};

const char * to_string(NavigationState state);

struct PlannerConfig
{
  std::string algorithm = "sector";  // "sector", "apf" or "follow_the_gap"
  double safe_distance = 0.65;
  double critical_distance = 0.30;
  double max_linear_speed = 0.22;
  double min_linear_speed = 0.05;
  double recovery_linear_speed = -0.10;
  double max_angular_speed = 1.2;
  double front_fov_deg = 120.0;
  double apf_attractive_gain = 1.0;
  double apf_repulsive_gain = 0.25;
  double apf_influence_distance = 1.2;
  double gap_threshold_distance = 0.80;
  // Step counts are LiDAR scans; the TurtleBot3 LDS publishes at 5 Hz.
  int stuck_timeout_steps = 25;     // ~5 s blocked before backing out
  int recovery_hold_steps = 12;     // ~2.4 s of reversing/turning
  int trap_timeout_steps = 35;      // ~7 s without goal progress
  int wall_follow_hold_steps = 28;  // ~5.6 s of wall following
  double robot_radius = 0.14;       // Burger is ~0.105 m, plus a safety margin
  double goal_tolerance = 0.22;
};

// Minimum range per sector, in this fixed order.
enum Sector { kRight = 0, kFrontRight, kFront, kFrontLeft, kLeft, kRear, kNumSectors };
extern const std::array<const char *, kNumSectors> kSectorNames;
using SectorDistances = std::array<double, kNumSectors>;

struct AvoidanceDecision
{
  double linear_x = 0.0;
  double angular_z = 0.0;
  NavigationState state = NavigationState::kClear;
  double min_front_distance = 0.0;
  double target_heading_rad = 0.0;
  bool has_sectors = false;  // false for PAUSED and empty scans
  SectorDistances sector_distances{};
};

struct ScanInput
{
  const std::vector<double> & ranges;
  double angle_min = 0.0;
  double angle_increment = 0.0;
  double range_min = 0.05;
  double range_max = 3.5;
};

// Replace NaN, inf and out-of-range readings with range_max (treated as free space).
std::vector<double> sanitize_scan(
  const std::vector<double> & ranges, double range_min, double range_max);

// Wrap an angle to [-pi, pi].
double normalize_angle(double angle_rad);

// Expand nearby obstacles sideways by the robot radius so narrow gaps close up.
std::vector<double> apply_footprint_bubbles(
  const std::vector<double> & cleaned, double angle_increment,
  double robot_radius, double range_max);

class AvoidancePlanner
{
public:
  explicit AvoidancePlanner(PlannerConfig config = PlannerConfig());

  // One control cycle: scan (+ optional goal relative to the robot) -> command.
  AvoidanceDecision plan(
    const ScanInput & scan, bool paused = false,
    std::optional<double> goal_distance = std::nullopt, double goal_angle_rad = 0.0);

  void reset_state();

  PlannerConfig & config() {return config_;}
  const PlannerConfig & config() const {return config_;}

private:
  SectorDistances compute_sectors(
    const std::vector<double> & cleaned, double angle_min, double angle_increment,
    double range_max) const;
  static std::pair<double, double> side_clearances(const SectorDistances & s);
  double select_turn_sign(double left_clearance, double right_clearance);
  void update_trap_detector(
    std::optional<double> goal_distance, double min_front, const SectorDistances & s);

  AvoidanceDecision plan_wall_follow(
    const std::vector<double> & cleaned, double angle_min, double angle_increment,
    const SectorDistances & s, double min_front);
  AvoidanceDecision plan_sector(
    const SectorDistances & s, double min_front, bool has_goal, double goal_angle_rad);
  AvoidanceDecision plan_apf(
    const std::vector<double> & cleaned, double angle_min, double angle_increment,
    const SectorDistances & s, double min_front, bool has_goal, double goal_angle_rad,
    std::optional<double> goal_distance);
  AvoidanceDecision plan_follow_the_gap(
    const std::vector<double> & cleaned, double angle_min, double angle_increment,
    const SectorDistances & s, double min_front, bool has_goal, double goal_angle_rad,
    double range_max);

  PlannerConfig config_;
  int stuck_counter_ = 0;
  int recovery_steps_left_ = 0;
  double recovery_turn_dir_ = 1.0;
  double preferred_turn_sign_ = 0.0;
  std::optional<double> best_goal_distance_;
  int no_progress_steps_ = 0;
  int wall_follow_steps_left_ = 0;
};

}  // namespace my_robot_navigation_cpp

#endif  // MY_ROBOT_NAVIGATION_CPP__AVOIDANCE_PLANNER_HPP_
