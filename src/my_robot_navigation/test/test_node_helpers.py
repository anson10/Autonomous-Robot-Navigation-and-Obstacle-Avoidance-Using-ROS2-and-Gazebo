# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Anson Antony

"""Tests for ROS-node helper functions (skipped when rclpy is not installed)."""

import math

import pytest

pytest.importorskip('rclpy')

from my_robot_navigation.obstacle_avoidance import (  # noqa: E402
    ObstacleAvoidanceNode,
    parse_waypoints,
    PLANNER_PARAMS,
    yaw_from_quaternion,
)


def test_parse_waypoints() -> None:
    """Waypoint strings parse into (x, y) tuples; blanks are ignored."""
    assert parse_waypoints('') == []
    assert parse_waypoints('1,2; -0.5, 3.25;') == [(1.0, 2.0), (-0.5, 3.25)]
    with pytest.raises(ValueError):
        parse_waypoints('1,2,3')


def test_yaw_from_quaternion() -> None:
    """A rotation of +90 degrees about z gives yaw = pi/2."""
    s = math.sin(math.pi / 4)
    assert yaw_from_quaternion(0.0, 0.0, s, s) == pytest.approx(math.pi / 2)
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)


def test_parameter_validation_rejects_bad_values() -> None:
    """Invalid algorithm names and inverted thresholds are rejected."""
    good = dict(PLANNER_PARAMS)
    assert ObstacleAvoidanceNode._validate(good) == ''
    assert 'algorithm' in ObstacleAvoidanceNode._validate({**good, 'algorithm': 'dwa'})
    assert 'critical_distance' in ObstacleAvoidanceNode._validate(
        {**good, 'critical_distance': 0.9}
    )
    assert 'recovery' in ObstacleAvoidanceNode._validate(
        {**good, 'recovery_linear_speed': 0.1}
    )
