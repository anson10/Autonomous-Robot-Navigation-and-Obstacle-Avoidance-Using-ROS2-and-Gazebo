// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Anson Antony

#include "my_robot_navigation_cpp/avoidance_planner.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace my_robot_navigation_cpp
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

// Sector bounds in degrees, same order as the Sector enum. 'rear' wraps around +-180.
constexpr std::array<std::pair<double, double>, kNumSectors> kSectorBounds{{
  {-90.0, -50.0}, {-50.0, -15.0}, {-15.0, 15.0}, {15.0, 50.0}, {50.0, 90.0}, {135.0, -135.0},
}};

double clamp(double value, double lo, double hi) {return std::max(lo, std::min(hi, value));}

double angle_at(double angle_min, std::size_t i, double angle_increment)
{
  return normalize_angle(angle_min + static_cast<double>(i) * angle_increment);
}

double resolved_increment(double angle_increment, std::size_t n)
{
  return angle_increment <= 0.0 ? (2.0 * kPi) / static_cast<double>(n) : angle_increment;
}

AvoidanceDecision make_decision(
  double linear_x, double angular_z, NavigationState state, double min_front,
  double target_heading, const SectorDistances & sectors)
{
  AvoidanceDecision d;
  d.linear_x = linear_x;
  d.angular_z = angular_z;
  d.state = state;
  d.min_front_distance = min_front;
  d.target_heading_rad = target_heading;
  d.has_sectors = true;
  d.sector_distances = sectors;
  return d;
}

// Python's round(x, 3): round half to even (the default FP rounding mode).
double round3(double x) {return std::nearbyint(x * 1000.0) / 1000.0;}

}  // namespace

const std::array<const char *, kNumSectors> kSectorNames{
  "right", "front_right", "front", "front_left", "left", "rear"};

const char * to_string(NavigationState state)
{
  switch (state) {
    case NavigationState::kPaused: return "PAUSED";
    case NavigationState::kClear: return "CLEAR";
    case NavigationState::kAvoiding: return "AVOIDING";
    case NavigationState::kCriticalStop: return "CRITICAL_STOP";
    case NavigationState::kRecovery: return "RECOVERY";
    case NavigationState::kWallFollow: return "WALL_FOLLOW";
    case NavigationState::kGoalReached: return "GOAL_REACHED";
  }
  return "UNKNOWN";
}

std::vector<double> sanitize_scan(
  const std::vector<double> & ranges, double range_min, double range_max)
{
  const double effective_min = std::max(range_min, 0.01);
  const double effective_max = std::max(range_max, effective_min);
  std::vector<double> cleaned;
  cleaned.reserve(ranges.size());
  for (double r : ranges) {
    if (std::isnan(r) || std::isinf(r) || r < effective_min || r > effective_max) {
      cleaned.push_back(effective_max);
    } else {
      cleaned.push_back(r);
    }
  }
  return cleaned;
}

double normalize_angle(double angle_rad)
{
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

std::vector<double> apply_footprint_bubbles(
  const std::vector<double> & cleaned, double angle_increment,
  double robot_radius, double range_max)
{
  if (cleaned.empty() || robot_radius <= 0.0) {
    return cleaned;
  }
  const auto n = static_cast<int>(cleaned.size());
  angle_increment = resolved_increment(angle_increment, cleaned.size());
  std::vector<double> bubbled = cleaned;
  for (int i = 0; i < n; ++i) {
    const double r = cleaned[i];
    if (r >= range_max || r <= 0.05) {
      continue;
    }
    // Cap at 45 deg so a very close obstacle can't blank the whole scan.
    const int half_span = std::min(
      static_cast<int>(std::ceil(std::atan2(robot_radius, r) / angle_increment)), n / 8);
    for (int offset = -half_span; offset <= half_span; ++offset) {
      const int idx = ((i + offset) % n + n) % n;  // Python-style modulo
      if (r < bubbled[idx]) {
        bubbled[idx] = r;
      }
    }
  }
  return bubbled;
}

AvoidancePlanner::AvoidancePlanner(PlannerConfig config)
: config_(std::move(config)) {}

void AvoidancePlanner::reset_state()
{
  stuck_counter_ = 0;
  recovery_steps_left_ = 0;
  preferred_turn_sign_ = 0.0;
  best_goal_distance_.reset();
  no_progress_steps_ = 0;
  wall_follow_steps_left_ = 0;
}

SectorDistances AvoidancePlanner::compute_sectors(
  const std::vector<double> & cleaned, double angle_min, double angle_increment,
  double range_max) const
{
  const double increment = resolved_increment(angle_increment, cleaned.size());
  SectorDistances sectors;
  for (int s = 0; s < kNumSectors; ++s) {
    const double min_rad = kSectorBounds[s].first * kDegToRad;
    const double max_rad = kSectorBounds[s].second * kDegToRad;
    double best = std::numeric_limits<double>::infinity();
    bool any = false;
    for (std::size_t i = 0; i < cleaned.size(); ++i) {
      const double angle = angle_at(angle_min, i, increment);
      const bool inside = min_rad <= max_rad ?
        (min_rad <= angle && angle <= max_rad) :
        (angle >= min_rad || angle <= max_rad);
      if (inside) {
        best = std::min(best, cleaned[i]);
        any = true;
      }
    }
    sectors[s] = any ? best : range_max;
  }
  return sectors;
}

std::pair<double, double> AvoidancePlanner::side_clearances(const SectorDistances & s)
{
  // The front-side sector counts more since that's where the robot is heading.
  return {0.65 * s[kFrontLeft] + 0.35 * s[kLeft], 0.65 * s[kFrontRight] + 0.35 * s[kRight]};
}

double AvoidancePlanner::select_turn_sign(double left_clearance, double right_clearance)
{
  const double diff = left_clearance - right_clearance;
  // Ignore left/right differences under 8 cm so the turn direction doesn't flip
  // back and forth on scan noise when the obstacle is roughly symmetric.
  constexpr double kHysteresisBand = 0.08;
  if (std::abs(diff) <= kHysteresisBand && preferred_turn_sign_ != 0.0) {
    return preferred_turn_sign_;
  }
  preferred_turn_sign_ = diff >= 0.0 ? 1.0 : -1.0;
  return preferred_turn_sign_;
}

void AvoidancePlanner::update_trap_detector(
  std::optional<double> goal_distance, double min_front, const SectorDistances & s)
{
  if (!goal_distance) {
    best_goal_distance_.reset();
    no_progress_steps_ = 0;
    return;
  }
  // A jump of more than 0.6 m means a new goal was set.
  if (!best_goal_distance_ || *goal_distance > *best_goal_distance_ + 0.60) {
    best_goal_distance_ = goal_distance;
    no_progress_steps_ = 0;
    return;
  }
  // Progress means getting at least 3 cm closer than the best so far.
  if (*goal_distance < *best_goal_distance_ - 0.03) {
    best_goal_distance_ = goal_distance;
    no_progress_steps_ = 0;
  } else if (min_front < config_.safe_distance * 1.15) {  // only count stalls near obstacles
    ++no_progress_steps_;
  }
  if (no_progress_steps_ >= config_.trap_timeout_steps && wall_follow_steps_left_ == 0) {
    const double left_space = s[kFrontLeft] + s[kLeft];
    const double right_space = s[kFrontRight] + s[kRight];
    recovery_turn_dir_ = left_space >= right_space ? 1.0 : -1.0;
    wall_follow_steps_left_ = config_.wall_follow_hold_steps;
    no_progress_steps_ = 0;
  }
}

AvoidanceDecision AvoidancePlanner::plan(
  const ScanInput & scan, bool paused, std::optional<double> goal_distance,
  double goal_angle_rad)
{
  if (paused) {
    AvoidanceDecision d;
    d.state = NavigationState::kPaused;
    d.min_front_distance = scan.range_max;
    return d;
  }
  const std::vector<double> cleaned = sanitize_scan(scan.ranges, scan.range_min, scan.range_max);
  if (cleaned.empty()) {
    AvoidanceDecision d;
    d.state = NavigationState::kCriticalStop;
    return d;
  }

  const SectorDistances sectors =
    compute_sectors(cleaned, scan.angle_min, scan.angle_increment, scan.range_max);
  const double min_front = std::min({sectors[kFrontRight], sectors[kFront], sectors[kFrontLeft]});
  const double rear_clearance = sectors[kRear];

  // Goal reached?
  if (goal_distance && *goal_distance <= config_.goal_tolerance) {
    reset_state();
    return make_decision(0.0, 0.0, NavigationState::kGoalReached, min_front, 0.0, sectors);
  }

  // Blocked for too long -> back out for a fixed number of steps.
  if (min_front < config_.critical_distance) {
    ++stuck_counter_;
  } else if (min_front >= config_.safe_distance) {
    stuck_counter_ = 0;
  }
  if (stuck_counter_ >= config_.stuck_timeout_steps && recovery_steps_left_ == 0) {
    const double left_space = sectors[kFrontLeft] + sectors[kLeft];
    const double right_space = sectors[kFrontRight] + sectors[kRight];
    recovery_turn_dir_ = left_space >= right_space ? 1.0 : -1.0;
    recovery_steps_left_ = std::max(1, config_.recovery_hold_steps);
  }
  if (recovery_steps_left_ > 0) {
    --recovery_steps_left_;
    if (recovery_steps_left_ == 0 || min_front >= config_.safe_distance) {
      stuck_counter_ = 0;
      recovery_steps_left_ = 0;
    }
    // Only reverse if nothing is close behind; otherwise just turn in place.
    const bool safe_to_reverse = rear_clearance > config_.critical_distance;
    const double rev_speed = safe_to_reverse ? config_.recovery_linear_speed : 0.0;
    const double angular = recovery_turn_dir_ * config_.max_angular_speed;
    return make_decision(
      rev_speed, angular, NavigationState::kRecovery, min_front,
      std::copysign(kPi / 2.0, angular), sectors);
  }

  // No goal progress for a while (e.g. stuck in a U-shaped pocket) -> follow the wall.
  update_trap_detector(goal_distance, min_front, sectors);
  if (wall_follow_steps_left_ > 0) {
    --wall_follow_steps_left_;
    return plan_wall_follow(cleaned, scan.angle_min, scan.angle_increment, sectors, min_front);
  }

  // Forget the preferred turn direction once we're well clear.
  if (min_front > config_.safe_distance * 1.15) {
    preferred_turn_sign_ = 0.0;
  }

  const bool has_goal = goal_distance.has_value();
  const double goal_ang = has_goal ? normalize_angle(goal_angle_rad) : 0.0;

  std::string algo = config_.algorithm;
  std::transform(
    algo.begin(), algo.end(), algo.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  if (algo == "apf") {
    return plan_apf(
      cleaned, scan.angle_min, scan.angle_increment, sectors, min_front, has_goal, goal_ang,
      goal_distance);
  }
  if (algo == "follow_the_gap") {
    return plan_follow_the_gap(
      cleaned, scan.angle_min, scan.angle_increment, sectors, min_front, has_goal, goal_ang,
      scan.range_max);
  }
  return plan_sector(sectors, min_front, has_goal, goal_ang);
}

AvoidanceDecision AvoidancePlanner::plan_wall_follow(
  const std::vector<double> & cleaned, double angle_min, double angle_increment,
  const SectorDistances & s, double min_front)
{
  const PlannerConfig & cfg = config_;
  if (min_front < cfg.critical_distance) {
    const double angular = recovery_turn_dir_ * cfg.max_angular_speed;
    return make_decision(
      0.0, angular, NavigationState::kWallFollow, min_front,
      std::copysign(kPi / 2.0, angular), s);
  }
  angle_increment = resolved_increment(angle_increment, cleaned.size());

  // Find the closest obstacle in the front 180 deg and drive along it (90 deg to it).
  double min_r = 999.0;
  double obs_angle = 0.0;
  for (std::size_t i = 0; i < cleaned.size(); ++i) {
    const double ang = angle_at(angle_min, i, angle_increment);
    if (std::abs(ang) <= kPi / 2.0 && cleaned[i] < min_r) {
      min_r = cleaned[i];
      obs_angle = ang;
    }
  }
  const double tangent_heading = normalize_angle(obs_angle + recovery_turn_dir_ * (kPi / 2.0));
  const double angular_z =
    clamp(1.4 * tangent_heading, -cfg.max_angular_speed, cfg.max_angular_speed);  // P-gain
  const double linear_x =
    std::max(cfg.min_linear_speed, 0.65 * cfg.max_linear_speed);  // slower along a wall
  return make_decision(
    linear_x, angular_z, NavigationState::kWallFollow, min_front, tangent_heading, s);
}

AvoidanceDecision AvoidancePlanner::plan_sector(
  const SectorDistances & s, double min_front, bool has_goal, double goal_angle_rad)
{
  const PlannerConfig & cfg = config_;
  const auto [left_clearance, right_clearance] = side_clearances(s);
  const double turn_sign = select_turn_sign(left_clearance, right_clearance);

  if (min_front < cfg.critical_distance) {
    return make_decision(
      0.0, turn_sign * cfg.max_angular_speed, NavigationState::kCriticalStop, min_front,
      turn_sign * (kPi / 3.0), s);
  }

  if (min_front < cfg.safe_distance) {
    const double ratio = clamp(
      (min_front - cfg.critical_distance) /
      std::max(cfg.safe_distance - cfg.critical_distance, 1e-4), 0.0, 1.0);
    const double linear_x =
      cfg.min_linear_speed + ratio * (cfg.max_linear_speed - cfg.min_linear_speed);
    const double urgency = 1.0 - ratio;
    const double asymmetry = (left_clearance - right_clearance) / std::max(cfg.safe_distance, 1e-4);
    const double goal_bias = has_goal ? 0.25 * goal_angle_rad : 0.0;  // weak pull to the goal
    // Turn at 45% of max rate at safe_distance, rising to 100% at
    // critical_distance, plus a bit extra toward whichever side is more open.
    const double raw_turn = turn_sign * (0.45 + 0.55 * urgency) + 0.3 * asymmetry + goal_bias;
    const double angular_z =
      clamp(raw_turn * cfg.max_angular_speed, -cfg.max_angular_speed, cfg.max_angular_speed);
    const double heading = (angular_z / std::max(cfg.max_angular_speed, 1e-4)) * (kPi / 3.0);
    return make_decision(linear_x, angular_z, NavigationState::kAvoiding, min_front, heading, s);
  }

  double angular_z = 0.0;
  double linear_x = 0.0;
  double target_heading = 0.0;
  if (has_goal) {
    const double wall_balance = (s[kLeft] - s[kRight]) * 0.08;  // stay centred
    const double desired_turn = 1.35 * goal_angle_rad + wall_balance;  // P-gain on bearing
    angular_z = clamp(desired_turn, -cfg.max_angular_speed, cfg.max_angular_speed);
    // Slow down while turning toward the goal, but never below 35% speed.
    const double turn_slowdown = std::max(0.35, std::cos(std::min(std::abs(goal_angle_rad), 1.2)));
    linear_x = std::max(cfg.min_linear_speed, cfg.max_linear_speed * turn_slowdown);
    target_heading = goal_angle_rad;
  } else {
    // No goal: drive straight, drifting gently toward the more open side.
    const double wall_balance = (s[kLeft] - s[kRight]) * 0.15;
    angular_z = clamp(wall_balance, -0.25 * cfg.max_angular_speed, 0.25 * cfg.max_angular_speed);
    linear_x = cfg.max_linear_speed;
    target_heading = angular_z * 0.5;
  }
  return make_decision(linear_x, angular_z, NavigationState::kClear, min_front, target_heading, s);
}

AvoidanceDecision AvoidancePlanner::plan_apf(
  const std::vector<double> & cleaned, double angle_min, double angle_increment,
  const SectorDistances & s, double min_front, bool has_goal, double goal_angle_rad,
  std::optional<double> goal_distance)
{
  // Goal pulls, nearby obstacles push. Close to the goal, repulsion is scaled down by
  // (goal_distance / influence)^2 (Ge & Cui, 2000), and obstacles farther away than
  // the goal are ignored; otherwise a goal next to a wall is never reached.
  const PlannerConfig & cfg = config_;
  angle_increment = resolved_increment(angle_increment, cleaned.size());
  const auto [left_clearance, right_clearance] = side_clearances(s);
  const double turn_sign = select_turn_sign(left_clearance, right_clearance);

  const double att_angle = has_goal ? goal_angle_rad : 0.0;
  double fx = cfg.apf_attractive_gain * std::cos(att_angle);
  double fy = cfg.apf_attractive_gain * std::sin(att_angle);

  double goal_scale = 1.0;
  double ignore_beyond = std::numeric_limits<double>::infinity();
  if (has_goal && goal_distance) {
    goal_scale = std::pow(std::min(1.0, *goal_distance / cfg.apf_influence_distance), 2);
    ignore_beyond = *goal_distance + cfg.robot_radius;
  }

  const double half_fov = cfg.front_fov_deg / 2.0 * kDegToRad;
  for (std::size_t i = 0; i < cleaned.size(); ++i) {
    const double angle = angle_at(angle_min, i, angle_increment);
    const double r = cleaned[i];
    if (std::abs(angle) > half_fov || r >= cfg.apf_influence_distance || r > ignore_beyond) {
      continue;
    }
    // Measure partly from the robot's edge, and clamp so 1/r^2 can't blow up.
    const double eff_r = std::max(r - 0.5 * cfg.robot_radius, 0.06);
    // Weighting each beam by its angular width makes the total force an integral over
    // the field of view, so it doesn't grow with LiDAR resolution.
    const double rep_mag = goal_scale * angle_increment *
      (cfg.apf_repulsive_gain * (1.0 / eff_r - 1.0 / cfg.apf_influence_distance) /
      std::pow(eff_r, 2));
    // Push mostly straight away from the obstacle, plus 15% sideways so an
    // obstacle dead ahead doesn't leave the forces cancelling out.
    const double rx = -std::cos(angle);
    const double ry = -std::sin(angle);
    const double tx = -turn_sign * std::sin(angle);
    const double ty = turn_sign * std::cos(angle);
    fx += rep_mag * (0.85 * rx + 0.15 * tx);
    fy += rep_mag * (0.85 * ry + 0.15 * ty);
  }

  const double heading = std::atan2(fy, fx);
  if (min_front < cfg.critical_distance) {
    return make_decision(
      0.0, turn_sign * cfg.max_angular_speed, NavigationState::kCriticalStop, min_front,
      heading, s);
  }
  const double angular_z =
    clamp(1.5 * heading, -cfg.max_angular_speed, cfg.max_angular_speed);  // P-gain
  const NavigationState state =
    min_front < cfg.safe_distance ? NavigationState::kAvoiding : NavigationState::kClear;
  // Slow down for sharp turns (min 20% speed) and near obstacles.
  double speed_scale = std::max(0.2, std::cos(std::min(std::abs(heading), kPi / 2.0)));
  if (min_front < cfg.safe_distance) {
    speed_scale *= min_front / cfg.safe_distance;
  }
  const double linear_x = std::max(
    cfg.min_linear_speed, std::min(cfg.max_linear_speed, cfg.max_linear_speed * speed_scale));
  return make_decision(linear_x, angular_z, state, min_front, heading, s);
}

AvoidanceDecision AvoidancePlanner::plan_follow_the_gap(
  const std::vector<double> & cleaned, double angle_min, double angle_increment,
  const SectorDistances & s, double min_front, bool has_goal, double goal_angle_rad,
  double range_max)
{
  const PlannerConfig & cfg = config_;
  angle_increment = resolved_increment(angle_increment, cleaned.size());
  // Inflate obstacles by the robot radius so gaps too narrow to pass are closed.
  const std::vector<double> bubbled =
    apply_footprint_bubbles(cleaned, angle_increment, cfg.robot_radius, range_max);

  using Sample = std::pair<double, double>;  // (angle, range)
  const double half_fov = cfg.front_fov_deg / 2.0 * kDegToRad;
  std::vector<Sample> fov;
  for (std::size_t i = 0; i < bubbled.size(); ++i) {
    const double angle = angle_at(angle_min, i, angle_increment);
    if (std::abs(angle) <= half_fov) {
      fov.emplace_back(angle, bubbled[i]);
    }
  }
  std::stable_sort(
    fov.begin(), fov.end(), [](const Sample & a, const Sample & b) {return a.first < b.first;});

  // Split the field of view into gaps: contiguous runs of free samples.
  std::vector<std::vector<Sample>> gaps;
  std::vector<Sample> current;
  for (const Sample & sample : fov) {
    if (sample.second >= cfg.gap_threshold_distance) {
      current.push_back(sample);
    } else if (!current.empty()) {
      gaps.push_back(std::move(current));
      current.clear();
    }
  }
  if (!current.empty()) {
    gaps.push_back(std::move(current));
  }
  if (gaps.empty()) {
    return plan_sector(s, min_front, has_goal, goal_angle_rad);
  }

  double target_angle = 0.0;
  if (has_goal) {
    // Pick the gap nearest the goal bearing (ties -> wider gap), then aim at the point
    // inside it closest to the goal, kept a few degrees in from the gap edge.
    const double goal_ref = clamp(goal_angle_rad, -half_fov, half_fov);
    auto offset = [goal_ref](const std::vector<Sample> & gap) {
        const double lo = gap.front().first;
        const double hi = gap.back().first;
        return (lo <= goal_ref && goal_ref <= hi) ?
               0.0 : std::min(std::abs(goal_ref - lo), std::abs(goal_ref - hi));
      };
    const auto best = std::min_element(
      gaps.begin(), gaps.end(), [&](const auto & a, const auto & b) {
        const double oa = round3(offset(a));
        const double ob = round3(offset(b));
        return oa != ob ? oa<ob : a.size()> b.size();
      });
    const double lo = best->front().first;
    const double hi = best->back().first;
    const double margin = std::min(10.0 * kDegToRad, (hi - lo) / 2.0);
    target_angle = std::max(lo + margin, std::min(hi - margin, goal_ref));
  } else {
    // Classic follow-the-gap: deepest point of the widest gap, biased to the centre.
    const auto widest = std::max_element(
      gaps.begin(), gaps.end(),
      [](const auto & a, const auto & b) {return a.size() < b.size();});
    const double center = (*widest)[widest->size() / 2].first;
    const auto deepest = std::max_element(
      widest->begin(), widest->end(), [center](const Sample & a, const Sample & b) {
        return a.second - 0.35 * std::abs(a.first - center) <
        b.second - 0.35 * std::abs(b.first - center);
      });
    target_angle = deepest->first;
  }

  if (min_front < cfg.critical_distance) {
    const double turn_sign = target_angle >= 0.0 ? 1.0 : -1.0;
    return make_decision(
      0.0, turn_sign * cfg.max_angular_speed, NavigationState::kCriticalStop, min_front,
      target_angle, s);
  }
  const double angular_z =
    clamp(1.4 * target_angle, -cfg.max_angular_speed, cfg.max_angular_speed);  // P-gain
  const NavigationState state =
    min_front < cfg.safe_distance ? NavigationState::kAvoiding : NavigationState::kClear;
  const double speed_factor =
    state == NavigationState::kAvoiding ? std::min(1.0, min_front / cfg.safe_distance) : 1.0;
  const double linear_x = std::max(cfg.min_linear_speed, cfg.max_linear_speed * speed_factor);
  return make_decision(linear_x, angular_z, state, min_front, target_angle, s);
}

}  // namespace my_robot_navigation_cpp
