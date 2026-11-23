# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Anson Antony

"""
Python access to the C++ planner.

CppAvoidancePlanner is a drop-in replacement for
my_robot_navigation.avoidance_planner.AvoidancePlanner: same constructor, same
plan() signature, same AvoidanceDecision result. The Python-side `config`
dataclass is copied into the C++ planner before every call, so changing it at
runtime (as the dashboard does) behaves the same for both implementations.
"""

import dataclasses
from typing import Optional, Sequence

from my_robot_navigation.avoidance_planner import (
    AvoidanceDecision,
    NavigationState,
    PlannerConfig,
)
from my_robot_navigation_cpp import _planner


class CppAvoidancePlanner:
    """AvoidancePlanner backed by the C++ implementation."""

    def __init__(self, config: Optional[PlannerConfig] = None) -> None:
        self.config = config or PlannerConfig()
        self._impl = _planner.AvoidancePlanner(self._to_cpp(self.config))

    @staticmethod
    def _to_cpp(config: PlannerConfig) -> '_planner.PlannerConfig':
        cpp = _planner.PlannerConfig()
        for f in dataclasses.fields(config):
            setattr(cpp, f.name, getattr(config, f.name))
        return cpp

    def reset_state(self) -> None:
        """Reset recovery, hysteresis and trap-detector state."""
        self._impl.reset_state()

    def plan(
        self,
        ranges: Sequence[float],
        angle_min: float = 0.0,
        angle_increment: float = 0.0,
        range_min: float = 0.05,
        range_max: float = 3.5,
        paused: bool = False,
        goal_distance: Optional[float] = None,
        goal_angle_rad: float = 0.0,
    ) -> AvoidanceDecision:
        """Run one control cycle in C++ and return the decision."""
        self._impl.config = self._to_cpp(self.config)
        linear_x, angular_z, state, min_front, heading, sectors = self._impl.plan(
            [float('inf') if r is None else float(r) for r in ranges],
            angle_min, angle_increment, range_min, range_max, paused,
            goal_distance, goal_angle_rad,
        )
        return AvoidanceDecision(
            linear_x=linear_x,
            angular_z=angular_z,
            state=NavigationState(state),
            min_front_distance=min_front,
            target_heading_rad=heading,
            sector_distances=dict(sectors),
        )
