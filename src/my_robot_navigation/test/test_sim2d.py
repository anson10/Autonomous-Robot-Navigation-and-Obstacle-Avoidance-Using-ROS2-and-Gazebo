# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Anson Antony

"""Tests for the 2D simulator, world loader and benchmark."""

import math
from pathlib import Path

from my_robot_navigation.avoidance_planner import PlannerConfig
from my_robot_navigation.benchmark import main as benchmark_main
from my_robot_navigation.benchmark import make_scenarios
from my_robot_navigation.sim2d import (
    Arena,
    Box,
    Circle,
    load_world,
    raycast,
    run_episode,
)
import pytest

WORLD = Path(__file__).resolve().parents[1] / 'worlds' / 'obstacle_arena.world'


def test_load_world_reads_pillars_and_walls() -> None:
    """The Gazebo arena has 7 cylindrical pillars and 4 box walls."""
    arena = load_world(str(WORLD))
    assert len(arena.circles) == 7
    assert len(arena.boxes) == 4
    xmin, ymin, xmax, ymax = arena.bounds
    assert xmin == pytest.approx(-3.075) and xmax == pytest.approx(3.075)
    assert ymin == pytest.approx(-3.075) and ymax == pytest.approx(3.075)


def test_raycast_hits_circle_and_box_at_expected_range() -> None:
    """Ray 0 (forward) hits a pillar; ray 180 (backward) hits a wall."""
    arena = Arena(circles=[Circle(2.0, 0.0, 0.5)], boxes=[Box(-3.0, -1.0, -2.0, 1.0)])
    scan = raycast(arena, 0.0, 0.0, 0.0, num_rays=360)
    assert scan[0] == pytest.approx(1.5)
    assert scan[180] == pytest.approx(2.0)
    assert math.isinf(scan[90])  # nothing to the left within range


def test_raycast_respects_robot_yaw() -> None:
    """Rotating the robot by +90 degrees moves the forward hit to ray 270."""
    arena = Arena(circles=[Circle(2.0, 0.0, 0.5)])
    scan = raycast(arena, 0.0, 0.0, math.pi / 2, num_rays=360)
    assert scan[270] == pytest.approx(1.5)
    assert math.isinf(scan[0])


def test_episode_reaches_goal_in_empty_arena() -> None:
    """With no obstacles in the way, every algorithm reaches the goal."""
    arena = Arena(boxes=[Box(-5, -5, -4.9, 5)])
    for algo in ('sector', 'apf', 'follow_the_gap'):
        res = run_episode(arena, PlannerConfig(algorithm=algo), (0.0, 0.0, 0.0), (2.0, 0.0))
        assert res.success, algo
        assert res.path_length_m == pytest.approx(2.0, abs=0.3)


def test_episode_detects_collision() -> None:
    """Driving straight into a wall with avoidance disabled counts as a collision."""
    arena = Arena(boxes=[Box(0.5, -1.0, 0.6, 1.0)])
    cfg = PlannerConfig(algorithm='sector', safe_distance=0.02, critical_distance=0.01)
    res = run_episode(arena, cfg, (0.0, 0.0, 0.0), (2.0, 0.0), timeout_s=10.0)
    assert res.collided and not res.success


def test_scenarios_are_reproducible() -> None:
    """Same seed gives identical start/goal pairs (fair comparison across algorithms)."""
    base = load_world(str(WORLD))
    a = make_scenarios(base, 3, seed=11, clutter=2, min_dist=2.0)
    b = make_scenarios(base, 3, seed=11, clutter=2, min_dist=2.0)
    assert [s['start'] for s in a] == [s['start'] for s in b]
    assert [s['goal'] for s in a] == [s['goal'] for s in b]
    for sc in a:
        assert base.clearance(*sc['goal']) >= 0.40
        assert len(sc['arena'].circles) == len(base.circles) + 2


def test_benchmark_cli_smoke(capsys, tmp_path) -> None:
    """The benchmark CLI runs end to end and writes a CSV."""
    out = tmp_path / 'results.csv'
    assert benchmark_main([
        '--world', str(WORLD), '--trials', '1', '--algorithms', 'sector',
        '--timeout', '20', '--csv', str(out),
    ]) == 0
    assert '| `sector` |' in capsys.readouterr().out
    assert out.read_text().startswith('algorithm,trial,success')
