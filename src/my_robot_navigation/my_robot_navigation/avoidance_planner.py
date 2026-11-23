# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Anson Antony

"""Obstacle avoidance planners (sector, APF, follow-the-gap), kept free of ROS imports."""

from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Dict, List, Optional, Sequence, Tuple


class NavigationState(str, Enum):
    """Finite state machine states for reactive and goal-directed navigation."""

    PAUSED = 'PAUSED'
    CLEAR = 'CLEAR'
    AVOIDING = 'AVOIDING'
    CRITICAL_STOP = 'CRITICAL_STOP'
    RECOVERY = 'RECOVERY'
    WALL_FOLLOW = 'WALL_FOLLOW'
    GOAL_REACHED = 'GOAL_REACHED'


@dataclass
class PlannerConfig:
    """Configuration parameters for the obstacle avoidance planner."""

    algorithm: str = 'sector'  # 'sector', 'apf', or 'follow_the_gap'
    safe_distance: float = 0.65
    critical_distance: float = 0.30
    max_linear_speed: float = 0.22
    min_linear_speed: float = 0.05
    recovery_linear_speed: float = -0.10
    max_angular_speed: float = 1.2
    front_fov_deg: float = 120.0
    apf_attractive_gain: float = 1.0
    apf_repulsive_gain: float = 0.25
    apf_influence_distance: float = 1.2
    gap_threshold_distance: float = 0.80
    # Step counts are LiDAR scans; the TurtleBot3 LDS publishes at 5 Hz.
    stuck_timeout_steps: int = 25     # ~5 s blocked before backing out
    recovery_hold_steps: int = 12     # ~2.4 s of reversing/turning
    trap_timeout_steps: int = 35      # ~7 s without goal progress
    wall_follow_hold_steps: int = 28  # ~5.6 s of wall following
    robot_radius: float = 0.14        # Burger is ~0.105 m, plus a safety margin
    goal_tolerance: float = 0.22


@dataclass
class AvoidanceDecision:
    """Computed velocity command and diagnostic telemetry for a control cycle."""

    linear_x: float
    angular_z: float
    state: NavigationState
    min_front_distance: float
    target_heading_rad: float
    sector_distances: Dict[str, float] = field(default_factory=dict)


def sanitize_scan(
    ranges: Sequence[float],
    range_min: float = 0.05,
    range_max: float = 3.5,
) -> List[float]:
    """Filter out NaN, Inf, zero, and out-of-range LiDAR values."""
    effective_min = max(range_min, 0.01)
    effective_max = max(range_max, effective_min)
    cleaned: List[float] = []
    for r in ranges:
        if r is None or math.isnan(r) or math.isinf(r) or r < effective_min:
            cleaned.append(effective_max)
        elif r > effective_max:
            cleaned.append(effective_max)
        else:
            cleaned.append(float(r))
    return cleaned


def normalize_angle(angle_rad: float) -> float:
    """Wrap angle in radians to [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def extract_sector_ranges(
    cleaned_ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    min_deg: float,
    max_deg: float,
) -> List[float]:
    """Return LiDAR range readings falling within [min_deg, max_deg]."""
    if not cleaned_ranges:
        return []
    n = len(cleaned_ranges)
    if angle_increment <= 0.0:
        angle_increment = (2.0 * math.pi) / n

    min_rad = math.radians(min_deg)
    max_rad = math.radians(max_deg)
    selected: List[float] = []

    for i, r in enumerate(cleaned_ranges):
        angle = normalize_angle(angle_min + i * angle_increment)
        if min_rad <= max_rad:
            if min_rad <= angle <= max_rad:
                selected.append(r)
        else:
            if angle >= min_rad or angle <= max_rad:
                selected.append(r)
    return selected


def apply_footprint_bubbles(
    cleaned_ranges: Sequence[float],
    angle_increment: float,
    robot_radius: float,
    range_max: float = 3.5,
) -> List[float]:
    """Expand nearby obstacles angularly by the robot footprint radius."""
    if not cleaned_ranges or robot_radius <= 0.0:
        return list(cleaned_ranges)
    n = len(cleaned_ranges)
    if angle_increment <= 0.0:
        angle_increment = (2.0 * math.pi) / n

    bubbled = list(cleaned_ranges)
    for i, r in enumerate(cleaned_ranges):
        if r >= range_max or r <= 0.05:
            continue
        half_span = min(
            int(math.ceil(math.atan2(robot_radius, r) / angle_increment)),
            n // 8,  # cap at 45 deg so a very close obstacle can't blank the whole scan
        )
        for offset in range(-half_span, half_span + 1):
            idx = (i + offset) % n
            if r < bubbled[idx]:
                bubbled[idx] = r
    return bubbled


class AvoidancePlanner:
    """Core reactive and goal-seeking obstacle avoidance planner."""

    SECTOR_BOUNDS: Dict[str, Tuple[float, float]] = {
        'right': (-90.0, -50.0),
        'front_right': (-50.0, -15.0),
        'front': (-15.0, 15.0),
        'front_left': (15.0, 50.0),
        'left': (50.0, 90.0),
        'rear': (135.0, -135.0),
    }

    def __init__(self, config: PlannerConfig | None = None) -> None:
        """Initialize the planner with the provided configuration."""
        self.config = config or PlannerConfig()
        self._stuck_counter = 0
        self._recovery_steps_left = 0
        self._recovery_turn_dir = 1.0
        self._preferred_turn_sign = 0.0
        self._best_goal_distance: Optional[float] = None
        self._no_progress_steps = 0
        self._wall_follow_steps_left = 0

    def reset_state(self) -> None:
        """Reset internal hysteresis, recovery, and trap escape counters."""
        self._stuck_counter = 0
        self._recovery_steps_left = 0
        self._preferred_turn_sign = 0.0
        self._best_goal_distance = None
        self._no_progress_steps = 0
        self._wall_follow_steps_left = 0

    def compute_sectors(
        self,
        cleaned_ranges: Sequence[float],
        angle_min: float = 0.0,
        angle_increment: float = 0.0,
        range_max: float = 3.5,
    ) -> Dict[str, float]:
        """Compute minimum obstacle distance in each of the 6 sectors (including rear)."""
        sectors: Dict[str, float] = {}
        for name, (low_deg, high_deg) in self.SECTOR_BOUNDS.items():
            vals = extract_sector_ranges(
                cleaned_ranges, angle_min, angle_increment, low_deg, high_deg
            )
            sectors[name] = min(vals) if vals else range_max
        return sectors

    @staticmethod
    def _side_clearances(sectors: Dict[str, float]) -> Tuple[float, float]:
        """Left/right free space; the front-side sector counts more since we're heading there."""
        left = 0.65 * sectors['front_left'] + 0.35 * sectors['left']
        right = 0.65 * sectors['front_right'] + 0.35 * sectors['right']
        return left, right

    def _select_turn_sign(
        self, left_clearance: float, right_clearance: float
    ) -> float:
        """Choose turn direction (+1 left, -1 right) with hysteresis deadband."""
        diff = left_clearance - right_clearance
        # Ignore left/right differences under 8 cm so the turn direction doesn't flip
        # back and forth on scan noise when the obstacle is roughly symmetric.
        hysteresis_band = 0.08
        if abs(diff) <= hysteresis_band and self._preferred_turn_sign != 0.0:
            return self._preferred_turn_sign
        self._preferred_turn_sign = 1.0 if diff >= 0.0 else -1.0
        return self._preferred_turn_sign

    def _update_trap_detector(
        self,
        goal_distance: Optional[float],
        min_front: float,
        sectors: Dict[str, float],
    ) -> None:
        """Detect local minima / U-trap stalls when no goal progress occurs."""
        if goal_distance is None:
            self._best_goal_distance = None
            self._no_progress_steps = 0
            return

        if (
            self._best_goal_distance is None
            or goal_distance > self._best_goal_distance + 0.60
        ):
            # A jump of more than 0.6 m means a new goal was set.
            self._best_goal_distance = goal_distance
            self._no_progress_steps = 0
            return

        # Progress means getting at least 3 cm closer than the best so far.
        if goal_distance < self._best_goal_distance - 0.03:
            self._best_goal_distance = goal_distance
            self._no_progress_steps = 0
        elif min_front < self.config.safe_distance * 1.15:  # only count stalls near obstacles
            self._no_progress_steps += 1

        if (
            self._no_progress_steps >= self.config.trap_timeout_steps
            and self._wall_follow_steps_left == 0
        ):
            left_space = sectors['front_left'] + sectors['left']
            right_space = sectors['front_right'] + sectors['right']
            self._recovery_turn_dir = 1.0 if left_space >= right_space else -1.0
            self._wall_follow_steps_left = self.config.wall_follow_hold_steps
            self._no_progress_steps = 0

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
        """Compute a velocity command and state from a LiDAR scan and optional goal."""
        if paused:
            return AvoidanceDecision(
                linear_x=0.0,
                angular_z=0.0,
                state=NavigationState.PAUSED,
                min_front_distance=range_max,
                target_heading_rad=0.0,
                sector_distances={},
            )

        cleaned = sanitize_scan(ranges, range_min=range_min, range_max=range_max)
        if not cleaned:
            return AvoidanceDecision(
                linear_x=0.0,
                angular_z=0.0,
                state=NavigationState.CRITICAL_STOP,
                min_front_distance=0.0,
                target_heading_rad=0.0,
                sector_distances={},
            )

        sectors = self.compute_sectors(
            cleaned,
            angle_min=angle_min,
            angle_increment=angle_increment,
            range_max=range_max,
        )
        min_front = min(
            sectors['front_right'],
            sectors['front'],
            sectors['front_left'],
        )
        rear_clearance = sectors.get('rear', range_max)

        # Goal reached?
        if (
            goal_distance is not None
            and goal_distance <= self.config.goal_tolerance
        ):
            self.reset_state()
            return AvoidanceDecision(
                linear_x=0.0,
                angular_z=0.0,
                state=NavigationState.GOAL_REACHED,
                min_front_distance=min_front,
                target_heading_rad=0.0,
                sector_distances=sectors,
            )

        # Blocked for too long -> back out for a fixed number of steps.
        if min_front < self.config.critical_distance:
            self._stuck_counter += 1
        elif min_front >= self.config.safe_distance:
            self._stuck_counter = 0

        if (
            self._stuck_counter >= self.config.stuck_timeout_steps
            and self._recovery_steps_left == 0
        ):
            left_space = sectors['front_left'] + sectors['left']
            right_space = sectors['front_right'] + sectors['right']
            self._recovery_turn_dir = 1.0 if left_space >= right_space else -1.0
            self._recovery_steps_left = max(1, self.config.recovery_hold_steps)

        if self._recovery_steps_left > 0:
            self._recovery_steps_left -= 1
            if (
                self._recovery_steps_left == 0
                or min_front >= self.config.safe_distance
            ):
                self._stuck_counter = 0
                self._recovery_steps_left = 0

            # Only reverse if nothing is close behind; otherwise just turn in place.
            safe_to_reverse = rear_clearance > self.config.critical_distance
            rev_speed = (
                self.config.recovery_linear_speed if safe_to_reverse else 0.0
            )
            angular = self._recovery_turn_dir * self.config.max_angular_speed
            return AvoidanceDecision(
                linear_x=rev_speed,
                angular_z=angular,
                state=NavigationState.RECOVERY,
                min_front_distance=min_front,
                target_heading_rad=math.copysign(math.pi / 2.0, angular),
                sector_distances=sectors,
            )

        # No goal progress for a while (e.g. stuck in a U-shaped pocket) -> follow the wall.
        self._update_trap_detector(goal_distance, min_front, sectors)
        if self._wall_follow_steps_left > 0:
            self._wall_follow_steps_left -= 1
            return self._plan_wall_follow(
                cleaned, angle_min, angle_increment, sectors, min_front
            )

        # Forget the preferred turn direction once we're well clear.
        if min_front > self.config.safe_distance * 1.15:
            self._preferred_turn_sign = 0.0

        has_goal = goal_distance is not None
        goal_ang = normalize_angle(goal_angle_rad) if has_goal else 0.0

        algo = self.config.algorithm.lower()
        if algo == 'apf':
            return self._plan_apf(
                cleaned,
                angle_min,
                angle_increment,
                sectors,
                min_front,
                has_goal=has_goal,
                goal_angle_rad=goal_ang,
                goal_distance=goal_distance,
            )
        if algo == 'follow_the_gap':
            return self._plan_follow_the_gap(
                cleaned,
                angle_min,
                angle_increment,
                sectors,
                min_front,
                has_goal=has_goal,
                goal_angle_rad=goal_ang,
                range_max=range_max,
            )
        return self._plan_sector(
            sectors,
            min_front,
            has_goal=has_goal,
            goal_angle_rad=goal_ang,
        )

    def _plan_wall_follow(
        self,
        cleaned_ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        sectors: Dict[str, float],
        min_front: float,
    ) -> AvoidanceDecision:
        """Tangential 90-degree contour following to escape U-shaped local minima."""
        cfg = self.config
        if min_front < cfg.critical_distance:
            angular = self._recovery_turn_dir * cfg.max_angular_speed
            return AvoidanceDecision(
                linear_x=0.0,
                angular_z=angular,
                state=NavigationState.WALL_FOLLOW,
                min_front_distance=min_front,
                target_heading_rad=math.copysign(math.pi / 2.0, angular),
                sector_distances=sectors,
            )

        n = len(cleaned_ranges)
        if angle_increment <= 0.0:
            angle_increment = (2.0 * math.pi) / n

        # Find the closest obstacle in the front 180 deg and drive along it (90 deg to it).
        min_r = 999.0
        obs_angle = 0.0
        for i, r in enumerate(cleaned_ranges):
            ang = normalize_angle(angle_min + i * angle_increment)
            if abs(ang) <= math.pi / 2.0 and r < min_r:
                min_r = r
                obs_angle = ang

        tangent_heading = normalize_angle(
            obs_angle + self._recovery_turn_dir * (math.pi / 2.0)
        )
        angular_z = max(
            -cfg.max_angular_speed,
            min(cfg.max_angular_speed, 1.4 * tangent_heading),  # P-gain on heading
        )
        linear_x = max(
            cfg.min_linear_speed,
            0.65 * cfg.max_linear_speed,  # slower while hugging a wall
        )
        return AvoidanceDecision(
            linear_x=linear_x,
            angular_z=angular_z,
            state=NavigationState.WALL_FOLLOW,
            min_front_distance=min_front,
            target_heading_rad=tangent_heading,
            sector_distances=sectors,
        )

    def _plan_sector(
        self,
        sectors: Dict[str, float],
        min_front: float,
        has_goal: bool = False,
        goal_angle_rad: float = 0.0,
    ) -> AvoidanceDecision:
        """Multi-sector proportional reactive obstacle avoidance with turn hysteresis."""
        cfg = self.config
        left_clearance, right_clearance = self._side_clearances(sectors)
        turn_sign = self._select_turn_sign(left_clearance, right_clearance)

        if min_front < cfg.critical_distance:
            angular_z = turn_sign * cfg.max_angular_speed
            return AvoidanceDecision(
                linear_x=0.0,
                angular_z=angular_z,
                state=NavigationState.CRITICAL_STOP,
                min_front_distance=min_front,
                target_heading_rad=turn_sign * (math.pi / 3.0),
                sector_distances=sectors,
            )

        if min_front < cfg.safe_distance:
            ratio = (min_front - cfg.critical_distance) / max(
                cfg.safe_distance - cfg.critical_distance, 1e-4
            )
            ratio = max(0.0, min(1.0, ratio))
            linear_x = (
                cfg.min_linear_speed
                + ratio * (cfg.max_linear_speed - cfg.min_linear_speed)
            )
            urgency = 1.0 - ratio
            asymmetry = (left_clearance - right_clearance) / max(
                cfg.safe_distance, 1e-4
            )
            goal_bias = 0.25 * goal_angle_rad if has_goal else 0.0  # weak pull to the goal
            # Turn at 45% of max rate at safe_distance, rising to 100% at
            # critical_distance, plus a bit extra toward whichever side is more open.
            raw_turn = (
                turn_sign * (0.45 + 0.55 * urgency)
                + 0.3 * asymmetry
                + goal_bias
            )
            angular_z = max(
                -cfg.max_angular_speed,
                min(cfg.max_angular_speed, raw_turn * cfg.max_angular_speed),
            )
            heading = (angular_z / max(cfg.max_angular_speed, 1e-4)) * (
                math.pi / 3.0
            )
            return AvoidanceDecision(
                linear_x=linear_x,
                angular_z=angular_z,
                state=NavigationState.AVOIDING,
                min_front_distance=min_front,
                target_heading_rad=heading,
                sector_distances=sectors,
            )

        if has_goal:
            wall_balance = (sectors['left'] - sectors['right']) * 0.08  # stay centred
            desired_turn = 1.35 * goal_angle_rad + wall_balance  # P-gain on bearing
            angular_z = max(
                -cfg.max_angular_speed,
                min(cfg.max_angular_speed, desired_turn),
            )
            # Slow down while turning toward the goal, but never below 35% speed.
            turn_slowdown = max(0.35, math.cos(min(abs(goal_angle_rad), 1.2)))
            linear_x = max(
                cfg.min_linear_speed,
                cfg.max_linear_speed * turn_slowdown,
            )
            target_heading = goal_angle_rad
        else:
            # No goal: drive straight, drifting gently toward the more open side.
            wall_balance = (sectors['left'] - sectors['right']) * 0.15
            angular_z = max(
                -0.25 * cfg.max_angular_speed,
                min(0.25 * cfg.max_angular_speed, wall_balance),
            )
            linear_x = cfg.max_linear_speed
            target_heading = angular_z * 0.5

        return AvoidanceDecision(
            linear_x=linear_x,
            angular_z=angular_z,
            state=NavigationState.CLEAR,
            min_front_distance=min_front,
            target_heading_rad=target_heading,
            sector_distances=sectors,
        )

    def _plan_apf(
        self,
        cleaned_ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        sectors: Dict[str, float],
        min_front: float,
        has_goal: bool = False,
        goal_angle_rad: float = 0.0,
        goal_distance: Optional[float] = None,
    ) -> AvoidanceDecision:
        """
        Artificial potential field: goal pulls, nearby obstacles push.

        Close to the goal, repulsion is scaled down by (goal_distance / influence)^2
        (Ge & Cui, 2000). Without that, a goal next to a wall is never reached: the
        wall's push cancels the goal's pull and the robot circles about 0.4 m away.
        Obstacles farther away than the goal are ignored too, since they can't be in
        the way.
        """
        cfg = self.config
        n = len(cleaned_ranges)
        if angle_increment <= 0.0:
            angle_increment = (2.0 * math.pi) / n

        left_clearance, right_clearance = self._side_clearances(sectors)
        turn_sign = self._select_turn_sign(left_clearance, right_clearance)

        att_angle = goal_angle_rad if has_goal else 0.0
        fx = cfg.apf_attractive_gain * math.cos(att_angle)
        fy = cfg.apf_attractive_gain * math.sin(att_angle)

        goal_scale = 1.0
        ignore_beyond = math.inf
        if has_goal and goal_distance is not None:
            goal_scale = min(1.0, goal_distance / cfg.apf_influence_distance) ** 2
            # Obstacles farther away than the goal cannot lie between robot and goal.
            ignore_beyond = goal_distance + cfg.robot_radius

        half_fov = math.radians(cfg.front_fov_deg / 2.0)
        for i, r in enumerate(cleaned_ranges):
            angle = normalize_angle(angle_min + i * angle_increment)
            if (
                abs(angle) > half_fov
                or r >= cfg.apf_influence_distance
                or r > ignore_beyond
            ):
                continue
            # Measure partly from the robot's edge, and clamp so 1/r^2 can't blow up.
            eff_r = max(r - 0.5 * cfg.robot_radius, 0.06)
            # Weighting each beam by its angular width (angle_increment) makes the
            # total force an integral over the field of view, so it does not grow
            # with LiDAR resolution: a wall pushes as hard at 360 beams as at 720.
            rep_mag = goal_scale * angle_increment * (
                cfg.apf_repulsive_gain
                * (1.0 / eff_r - 1.0 / cfg.apf_influence_distance)
                / (eff_r ** 2)
            )
            # Push mostly straight away from the obstacle, plus 15% sideways so an
            # obstacle dead ahead doesn't leave the forces cancelling out.
            rx_vec = -math.cos(angle)
            ry_vec = -math.sin(angle)
            tx_vec = -turn_sign * math.sin(angle)
            ty_vec = turn_sign * math.cos(angle)
            fx += rep_mag * (0.85 * rx_vec + 0.15 * tx_vec)
            fy += rep_mag * (0.85 * ry_vec + 0.15 * ty_vec)

        heading = math.atan2(fy, fx)
        if min_front < cfg.critical_distance:
            return AvoidanceDecision(
                linear_x=0.0,
                angular_z=turn_sign * cfg.max_angular_speed,
                state=NavigationState.CRITICAL_STOP,
                min_front_distance=min_front,
                target_heading_rad=heading,
                sector_distances=sectors,
            )

        angular_z = max(
            -cfg.max_angular_speed,
            min(cfg.max_angular_speed, 1.5 * heading),  # P-gain on heading
        )
        state = (
            NavigationState.AVOIDING
            if min_front < cfg.safe_distance
            else NavigationState.CLEAR
        )
        # Slow down for sharp turns (min 20% speed) and near obstacles.
        speed_scale = max(0.2, math.cos(min(abs(heading), math.pi / 2.0)))
        if min_front < cfg.safe_distance:
            speed_scale *= min_front / cfg.safe_distance
        linear_x = max(
            cfg.min_linear_speed,
            min(cfg.max_linear_speed, cfg.max_linear_speed * speed_scale),
        )
        return AvoidanceDecision(
            linear_x=linear_x,
            angular_z=angular_z,
            state=state,
            min_front_distance=min_front,
            target_heading_rad=heading,
            sector_distances=sectors,
        )

    def _plan_follow_the_gap(
        self,
        cleaned_ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        sectors: Dict[str, float],
        min_front: float,
        has_goal: bool = False,
        goal_angle_rad: float = 0.0,
        range_max: float = 3.5,
    ) -> AvoidanceDecision:
        """Follow-the-gap: inflate obstacles by the robot radius, then steer into a free gap."""
        cfg = self.config
        n = len(cleaned_ranges)
        if angle_increment <= 0.0:
            angle_increment = (2.0 * math.pi) / n

        # Expand obstacles by robot_radius so narrow unpassable gaps are filtered out
        bubbled = apply_footprint_bubbles(
            cleaned_ranges,
            angle_increment=angle_increment,
            robot_radius=cfg.robot_radius,
            range_max=range_max,
        )

        half_fov = math.radians(cfg.front_fov_deg / 2.0)
        fov_samples: List[Tuple[float, float]] = []
        for i, r in enumerate(bubbled):
            angle = normalize_angle(angle_min + i * angle_increment)
            if abs(angle) <= half_fov:
                fov_samples.append((angle, r))

        fov_samples.sort(key=lambda pair: pair[0])

        # Split the field of view into gaps: contiguous runs of free samples.
        gaps: List[List[Tuple[float, float]]] = []
        current: List[Tuple[float, float]] = []
        for angle, r in fov_samples:
            if r >= cfg.gap_threshold_distance:
                current.append((angle, r))
            elif current:
                gaps.append(current)
                current = []
        if current:
            gaps.append(current)

        if not gaps:
            return self._plan_sector(
                sectors, min_front, has_goal=has_goal, goal_angle_rad=goal_angle_rad
            )

        if has_goal:
            # Goal-directed: pick the gap nearest the goal bearing (ties -> wider gap),
            # then aim at the point inside it closest to the goal, kept a few degrees
            # in from the gap edge.
            goal_ref = max(-half_fov, min(half_fov, goal_angle_rad))

            def offset(gap: List[Tuple[float, float]]) -> float:
                lo, hi = gap[0][0], gap[-1][0]
                return 0.0 if lo <= goal_ref <= hi else min(
                    abs(goal_ref - lo), abs(goal_ref - hi)
                )

            best_gap = min(gaps, key=lambda g: (round(offset(g), 3), -len(g)))
            lo, hi = best_gap[0][0], best_gap[-1][0]
            margin = min(math.radians(10.0), (hi - lo) / 2.0)
            target_angle = max(lo + margin, min(hi - margin, goal_ref))
        else:
            # Classic follow-the-gap: deepest point of the widest gap, biased to centre.
            best_gap = max(gaps, key=len)
            center = best_gap[len(best_gap) // 2][0]
            target_angle, _ = max(
                best_gap,
                key=lambda item: item[1] - 0.35 * abs(item[0] - center),
            )

        if min_front < cfg.critical_distance:
            turn_sign = 1.0 if target_angle >= 0.0 else -1.0
            return AvoidanceDecision(
                linear_x=0.0,
                angular_z=turn_sign * cfg.max_angular_speed,
                state=NavigationState.CRITICAL_STOP,
                min_front_distance=min_front,
                target_heading_rad=target_angle,
                sector_distances=sectors,
            )

        angular_z = max(
            -cfg.max_angular_speed,
            min(cfg.max_angular_speed, 1.4 * target_angle),  # P-gain on heading
        )
        state = (
            NavigationState.AVOIDING
            if min_front < cfg.safe_distance
            else NavigationState.CLEAR
        )
        speed_factor = (
            min(1.0, min_front / cfg.safe_distance)
            if state == NavigationState.AVOIDING
            else 1.0
        )
        linear_x = max(
            cfg.min_linear_speed,
            cfg.max_linear_speed * speed_factor,
        )
        return AvoidanceDecision(
            linear_x=linear_x,
            angular_z=angular_z,
            state=state,
            min_front_distance=min_front,
            target_heading_rad=target_angle,
            sector_distances=sectors,
        )
