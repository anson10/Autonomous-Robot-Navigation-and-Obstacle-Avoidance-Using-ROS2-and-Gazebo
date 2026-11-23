# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Anson Antony

"""
The C++ planner must make the same decisions as the Python reference.

Both planners see the same sequence of scans from closed-loop 2D episodes (the
Python planner drives; the C++ planner is fed the same inputs), so stateful
behaviour such as recovery, hysteresis and the trap detector is compared too.
"""

import math
from pathlib import Path

from my_robot_navigation.avoidance_planner import AvoidancePlanner, PlannerConfig
from my_robot_navigation.benchmark import default_world_path, make_scenarios
from my_robot_navigation.sim2d import load_world, raycast
from my_robot_navigation_cpp import CppAvoidancePlanner
import pytest

TOL = 1e-9


def _world() -> Path:
    path = default_world_path()
    if not path.exists():
        pytest.skip('obstacle_arena.world not found')
    return path


@pytest.mark.parametrize('algorithm', ['sector', 'apf', 'follow_the_gap'])
def test_closed_loop_parity(algorithm: str) -> None:
    """Same commands, states and sector distances on every step of 10 episodes."""
    base = load_world(str(_world()))
    steps_checked = 0
    for sc in make_scenarios(base, 10, seed=123, clutter=4, min_dist=2.5):
        py = AvoidancePlanner(PlannerConfig(algorithm=algorithm))
        cpp = CppAvoidancePlanner(PlannerConfig(algorithm=algorithm))
        x, y, yaw = sc['start']
        gx, gy = sc['goal']
        for _ in range(400):
            dist = math.hypot(gx - x, gy - y)
            ang = math.atan2(gy - y, gx - x) - yaw
            scan = raycast(sc['arena'], x, y, yaw)
            args = (scan, 0.0, 2 * math.pi / len(scan), 0.12, 3.5, False, dist, ang)
            a, b = py.plan(*args), cpp.plan(*args)
            assert a.state == b.state
            assert a.linear_x == pytest.approx(b.linear_x, abs=TOL)
            assert a.angular_z == pytest.approx(b.angular_z, abs=TOL)
            assert a.target_heading_rad == pytest.approx(b.target_heading_rad, abs=TOL)
            assert a.sector_distances == pytest.approx(b.sector_distances, abs=TOL)
            steps_checked += 1
            if a.state.value == 'GOAL_REACHED':
                break
            x += a.linear_x * math.cos(yaw) * 0.1
            y += a.linear_x * math.sin(yaw) * 0.1
            yaw = math.atan2(math.sin(yaw + a.angular_z * 0.1), math.cos(yaw + a.angular_z * 0.1))
    assert steps_checked > 500


def test_edge_case_scans() -> None:
    """NaN/inf/negative readings, odd beam counts and no-goal mode agree too."""
    nan, inf = float('nan'), float('inf')
    scans = [
        [nan, inf, -1.0, 0.0, 0.3, 0.2] * 60,
        [0.4] * 359,
        [0.25 if i % 7 else inf for i in range(720)],
        [],
    ]
    for algorithm in ('sector', 'apf', 'follow_the_gap'):
        py = AvoidancePlanner(PlannerConfig(algorithm=algorithm))
        cpp = CppAvoidancePlanner(PlannerConfig(algorithm=algorithm))
        for scan in scans:
            for goal in (None, 1.0):
                args = (scan, -math.pi, 0.0, 0.12, 3.5, False, goal, 0.7)
                a, b = py.plan(*args), cpp.plan(*args)
                assert a.state == b.state
                assert a.linear_x == pytest.approx(b.linear_x, abs=TOL)
                assert a.angular_z == pytest.approx(b.angular_z, abs=TOL)
