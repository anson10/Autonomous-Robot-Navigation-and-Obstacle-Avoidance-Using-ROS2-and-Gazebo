# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Anson Antony

"""Unit tests for the obstacle avoidance planner algorithms and LiDAR sanitization."""

import math
from typing import List

from my_robot_navigation.avoidance_planner import (
    AvoidancePlanner,
    NavigationState,
    PlannerConfig,
    sanitize_scan,
)
import pytest


def make_360_scan(default_range: float = 3.0) -> List[float]:
    """Create a 360-sample LiDAR scan initialized to default_range."""
    return [default_range] * 360


def test_sanitize_scan_filters_nan_inf_and_zeros() -> None:
    """Ensure NaN, Inf, negative, and zero values are clamped to range_max."""
    raw = [float('nan'), float('inf'), -1.0, 0.0, 0.01, 0.5, 5.0]
    cleaned = sanitize_scan(raw, range_min=0.05, range_max=3.5)
    assert cleaned == [3.5, 3.5, 3.5, 3.5, 3.5, 0.5, 3.5]


def test_clear_path_moves_forward_at_max_speed() -> None:
    """When all sectors are clear, robot moves forward in CLEAR state."""
    planner = AvoidancePlanner(PlannerConfig(algorithm='sector'))
    scan = make_360_scan(3.0)
    decision = planner.plan(scan)
    assert decision.state == NavigationState.CLEAR
    assert decision.linear_x == pytest.approx(0.22)
    assert abs(decision.angular_z) < 0.05


def test_obstacle_on_right_turns_left() -> None:
    """An obstacle in the front-right sector should steer the robot left (+angular_z)."""
    planner = AvoidancePlanner(PlannerConfig(algorithm='sector'))
    scan = make_360_scan(3.0)
    # Front-right is angles 310..345 (-50 to -15 deg)
    for idx in range(315, 345):
        scan[idx] = 0.45
    decision = planner.plan(scan)
    assert decision.state == NavigationState.AVOIDING
    assert 0.05 < decision.linear_x < 0.22
    assert decision.angular_z > 0.0


def test_obstacle_on_left_turns_right() -> None:
    """An obstacle in the front-left sector should steer the robot right (-angular_z)."""
    planner = AvoidancePlanner(PlannerConfig(algorithm='sector'))
    scan = make_360_scan(3.0)
    # Front-left is angles 15..50 deg
    for idx in range(15, 45):
        scan[idx] = 0.45
    decision = planner.plan(scan)
    assert decision.state == NavigationState.AVOIDING
    assert decision.angular_z < 0.0


def test_critical_obstacle_stops_linear_motion() -> None:
    """An obstacle closer than critical_distance must stop forward motion immediately."""
    planner = AvoidancePlanner(PlannerConfig(algorithm='sector'))
    scan = make_360_scan(3.0)
    scan[0] = 0.20
    decision = planner.plan(scan)
    assert decision.state == NavigationState.CRITICAL_STOP
    assert decision.linear_x == 0.0
    assert abs(decision.angular_z) == pytest.approx(1.2)


def test_stuck_recovery_triggers_after_timeout() -> None:
    """Repeated critical blockages trigger RECOVERY reverse maneuver."""
    cfg = PlannerConfig(algorithm='sector', stuck_timeout_steps=3)
    planner = AvoidancePlanner(cfg)
    scan = make_360_scan(3.0)
    scan[0] = 0.18

    planner.plan(scan)
    planner.plan(scan)
    decision = planner.plan(scan)

    assert decision.state == NavigationState.RECOVERY
    assert decision.linear_x < 0.0
    assert abs(decision.angular_z) > 0.0


def test_paused_state_outputs_zero_velocity() -> None:
    """When paused is True, the planner outputs zero linear and angular velocity."""
    planner = AvoidancePlanner()
    scan = make_360_scan(0.25)
    decision = planner.plan(scan, paused=True)
    assert decision.state == NavigationState.PAUSED
    assert decision.linear_x == 0.0
    assert decision.angular_z == 0.0


def test_apf_algorithm_repels_from_front_right_obstacle() -> None:
    """APF planner computes a positive (leftward) angular velocity for right obstacles."""
    planner = AvoidancePlanner(PlannerConfig(algorithm='apf'))
    scan = make_360_scan(3.0)
    for idx in range(320, 350):
        scan[idx] = 0.45
    decision = planner.plan(scan, angle_min=0.0, angle_increment=2 * math.pi / 360)
    assert decision.state == NavigationState.AVOIDING
    assert decision.angular_z > 0.0
    assert decision.linear_x > 0.0


def test_follow_the_gap_steers_toward_open_gap() -> None:
    """Follow-the-Gap steers toward the open left side when front and right are blocked."""
    planner = AvoidancePlanner(PlannerConfig(algorithm='follow_the_gap'))
    scan = make_360_scan(3.0)
    # Block front and front-right (-60 to +10 deg), leave +15 to +60 deg open
    for idx in list(range(300, 360)) + list(range(0, 11)):
        scan[idx] = 0.45
    decision = planner.plan(scan, angle_min=0.0, angle_increment=2 * math.pi / 360)
    assert decision.state == NavigationState.AVOIDING
    assert decision.target_heading_rad > 0.0
    assert decision.angular_z > 0.0


def test_goal_seeking_steers_toward_goal_and_stops_when_reached() -> None:
    """Goal-aware planner steers toward goal_angle_rad and transitions to GOAL_REACHED."""
    planner = AvoidancePlanner(PlannerConfig(algorithm='apf', goal_tolerance=0.25))
    scan = make_360_scan(3.0)

    # Goal is at +45 degrees, 1.5m away
    moving = planner.plan(
        scan,
        angle_min=0.0,
        angle_increment=2 * math.pi / 360,
        goal_distance=1.5,
        goal_angle_rad=math.pi / 4,
    )
    assert moving.state == NavigationState.CLEAR
    assert moving.angular_z > 0.0

    # Goal reached within tolerance
    arrived = planner.plan(
        scan,
        angle_min=0.0,
        angle_increment=2 * math.pi / 360,
        goal_distance=0.15,
        goal_angle_rad=0.1,
    )
    assert arrived.state == NavigationState.GOAL_REACHED
    assert arrived.linear_x == 0.0
    assert arrived.angular_z == 0.0


def test_rear_obstacle_prevents_reverse_collision_during_recovery() -> None:
    """With the rear sector blocked, RECOVERY rotates in place (linear_x == 0)."""
    cfg = PlannerConfig(
        algorithm='sector', stuck_timeout_steps=2, recovery_hold_steps=4
    )
    planner = AvoidancePlanner(cfg)
    scan = make_360_scan(3.0)
    scan[0] = 0.18    # Front blocked
    scan[180] = 0.18  # Rear also blocked

    planner.plan(scan)
    decision = planner.plan(scan)
    assert decision.state == NavigationState.RECOVERY
    assert decision.linear_x == 0.0
    assert abs(decision.angular_z) > 0.0


def test_latched_recovery_holds_across_multiple_steps() -> None:
    """RECOVERY remains latched for recovery_hold_steps even if front slightly clears."""
    cfg = PlannerConfig(
        algorithm='sector', stuck_timeout_steps=2, recovery_hold_steps=3
    )
    planner = AvoidancePlanner(cfg)
    blocked = make_360_scan(3.0)
    blocked[0] = 0.18

    planner.plan(blocked)
    d1 = planner.plan(blocked)
    assert d1.state == NavigationState.RECOVERY

    # Next step: front is slightly above critical_distance (0.35m < safe_distance)
    slightly_clear = make_360_scan(3.0)
    slightly_clear[0] = 0.35
    d2 = planner.plan(slightly_clear)
    assert d2.state == NavigationState.RECOVERY


def test_u_trap_local_minimum_triggers_tangential_wall_following() -> None:
    """Stalled goal progress inside a U-shaped obstacle pocket triggers WALL_FOLLOW escape."""
    cfg = PlannerConfig(
        algorithm='apf',
        trap_timeout_steps=3,
        wall_follow_hold_steps=4,
    )
    planner = AvoidancePlanner(cfg)
    u_trap_scan = make_360_scan(3.0)
    for idx in list(range(320, 360)) + list(range(0, 41)):
        u_trap_scan[idx] = 0.45

    # Simulate 4 steps where goal_distance fails to decrease
    planner.plan(u_trap_scan, goal_distance=2.0, goal_angle_rad=0.0)
    planner.plan(u_trap_scan, goal_distance=2.0, goal_angle_rad=0.0)
    planner.plan(u_trap_scan, goal_distance=2.0, goal_angle_rad=0.0)
    decision = planner.plan(u_trap_scan, goal_distance=2.0, goal_angle_rad=0.0)

    assert decision.state == NavigationState.WALL_FOLLOW
    assert decision.linear_x > 0.0
    assert abs(decision.angular_z) > 0.0


def test_turn_hysteresis_prevents_symmetric_oscillation() -> None:
    """Turn hysteresis maintains consistent turn sign despite tiny left/right fluctuations."""
    planner = AvoidancePlanner(PlannerConfig(algorithm='sector'))

    # Step 1: Slightly more room on left -> turns left (+angular_z)
    scan1 = make_360_scan(3.0)
    scan1[0] = 0.45
    scan1[30] = 0.90   # front-left
    scan1[330] = 0.70  # front-right
    d1 = planner.plan(scan1)
    assert d1.angular_z > 0.0

    # Step 2: Tiny 0.02m fluctuation favoring right (within 0.08m hysteresis deadband)
    scan2 = make_360_scan(3.0)
    scan2[0] = 0.45
    scan2[30] = 0.79
    scan2[330] = 0.81
    d2 = planner.plan(scan2)
    # Must keep turning left instead of chattering right
    assert d2.angular_z > 0.0


def test_apf_reaches_goal_in_front_of_wall() -> None:
    """APF still reaches a goal that sits just in front of a wall."""
    planner = AvoidancePlanner(PlannerConfig(algorithm='apf'))
    scan = make_360_scan(3.0)
    for idx in list(range(320, 360)) + list(range(0, 41)):
        scan[idx] = 0.70  # wall 0.7 m ahead, spanning +/-40 degrees
    decision = planner.plan(
        scan,
        angle_min=0.0,
        angle_increment=2 * math.pi / 360,
        goal_distance=0.40,
        goal_angle_rad=0.0,
    )
    assert decision.linear_x > 0.0
    assert abs(decision.angular_z) < 0.1


def test_follow_the_gap_prefers_gap_towards_goal() -> None:
    """With a wide gap on the left and a narrower one towards the goal, go right."""
    planner = AvoidancePlanner(PlannerConfig(algorithm='follow_the_gap'))
    scan = make_360_scan(0.50)
    for idx in range(5, 66):  # wide open gap: +5..+65 degrees
        scan[idx] = 3.0
    for idx in range(305, 351):  # narrower open gap: -55..-10 degrees
        scan[idx] = 3.0
    # (Each gap shrinks by ~16 degrees per side once robot-radius bubbles are added;
    # a gap much narrower than ~35 degrees at 0.5 m is correctly treated as closed.)
    decision = planner.plan(
        scan,
        angle_min=0.0,
        angle_increment=2 * math.pi / 360,
        goal_distance=3.0,
        goal_angle_rad=math.radians(-30.0),
    )
    assert decision.target_heading_rad < 0.0
    assert decision.angular_z < 0.0
