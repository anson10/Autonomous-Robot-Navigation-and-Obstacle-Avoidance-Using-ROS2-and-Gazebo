# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Anson Antony

"""
Lightweight 2D kinematic simulator for testing the planner without ROS or Gazebo.

The arena is read straight from the Gazebo ``.world`` file, so the 2D demo, the
benchmark and the Gazebo simulation all use the same obstacle layout.

Limitations:
- unicycle kinematics with no inertia, wheel slip or sensor noise;
- obstacles are circles (SDF cylinders) and axis-aligned boxes (SDF boxes);
  model/link rotations are ignored.
"""

from dataclasses import dataclass, field
import math
import random
from typing import List, Optional, Sequence, Tuple
import xml.etree.ElementTree as ET

from my_robot_navigation.avoidance_planner import (
    AvoidancePlanner,
    NavigationState,
    normalize_angle,
    PlannerConfig,
)

# TurtleBot3 Burger footprint radius (m), used for collision checks.
BURGER_RADIUS = 0.105


@dataclass
class Circle:
    """Circular obstacle (SDF cylinder seen from above)."""

    x: float
    y: float
    r: float


@dataclass
class Box:
    """Axis-aligned rectangular obstacle."""

    xmin: float
    ymin: float
    xmax: float
    ymax: float


@dataclass
class Arena:
    """Collection of obstacles plus the bounding region used for sampling."""

    circles: List[Circle] = field(default_factory=list)
    boxes: List[Box] = field(default_factory=list)
    bounds: Tuple[float, float, float, float] = (-3.0, -3.0, 3.0, 3.0)

    def copy(self) -> 'Arena':
        """Return a shallow copy with independent obstacle lists."""
        return Arena(list(self.circles), list(self.boxes), self.bounds)

    def clearance(self, x: float, y: float) -> float:
        """Distance from point (x, y) to the nearest obstacle surface."""
        best = math.inf
        for c in self.circles:
            best = min(best, math.hypot(x - c.x, y - c.y) - c.r)
        for b in self.boxes:
            dx = max(b.xmin - x, 0.0, x - b.xmax)
            dy = max(b.ymin - y, 0.0, y - b.ymax)
            if dx == 0.0 and dy == 0.0:
                return 0.0
            best = min(best, math.hypot(dx, dy))
        return best


def _pose_xy(elem: Optional[ET.Element]) -> Tuple[float, float]:
    if elem is None or elem.find('pose') is None:
        return 0.0, 0.0
    vals = [float(v) for v in elem.find('pose').text.split()]
    return vals[0], vals[1]


def load_world(path: str) -> Arena:
    """Parse cylinders and boxes from a Gazebo SDF world into an Arena."""
    root = ET.parse(path).getroot()
    arena = Arena()
    for model in root.iter('model'):
        mx, my = _pose_xy(model)
        for link in model.findall('link'):
            lx, ly = _pose_xy(link)
            geom = link.find('collision/geometry')
            if geom is None:
                continue
            cx, cy = mx + lx, my + ly
            cyl = geom.find('cylinder')
            box = geom.find('box')
            if cyl is not None:
                arena.circles.append(
                    Circle(cx, cy, float(cyl.find('radius').text))
                )
            elif box is not None:
                sx, sy, _ = (float(v) for v in box.find('size').text.split())
                arena.boxes.append(
                    Box(cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2)
                )
    if arena.boxes:
        arena.bounds = (
            min(b.xmin for b in arena.boxes),
            min(b.ymin for b in arena.boxes),
            max(b.xmax for b in arena.boxes),
            max(b.ymax for b in arena.boxes),
        )
    return arena


def _ray_box(
    ox: float, oy: float, dx: float, dy: float, b: Box, t_max: float
) -> float:
    """Slab-method ray/AABB intersection; returns hit distance or t_max."""
    t0, t1 = -math.inf, math.inf
    for o, d, lo, hi in ((ox, dx, b.xmin, b.xmax), (oy, dy, b.ymin, b.ymax)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return t_max
            continue
        ta, tb = (lo - o) / d, (hi - o) / d
        if ta > tb:
            ta, tb = tb, ta
        t0, t1 = max(t0, ta), min(t1, tb)
        if t0 > t1:
            return t_max
    if 0.0 < t0 < t_max:
        return t0
    return t_max


def raycast(
    arena: Arena,
    x: float,
    y: float,
    yaw: float,
    num_rays: int = 360,
    range_max: float = 3.5,
) -> List[float]:
    """Simulate a 360-degree LiDAR (angle_min = 0, counter-clockwise)."""
    ranges: List[float] = []
    for i in range(num_rays):
        ang = yaw + 2.0 * math.pi * i / num_rays
        dx, dy = math.cos(ang), math.sin(ang)
        t_min = range_max
        for c in arena.circles:
            ox, oy = x - c.x, y - c.y
            b = ox * dx + oy * dy
            disc = b * b - (ox * ox + oy * oy - c.r * c.r)
            if disc >= 0.0:
                t = -b - math.sqrt(disc)
                if 0.0 < t < t_min:
                    t_min = t
        for box in arena.boxes:
            t_min = _ray_box(x, y, dx, dy, box, t_min)
        ranges.append(t_min if t_min < range_max else math.inf)
    return ranges


@dataclass
class EpisodeResult:
    """Outcome of one start-to-goal run."""

    algorithm: str
    success: bool
    collided: bool
    timed_out: bool
    sim_time_s: float
    path_length_m: float
    straight_line_m: float
    min_clearance_m: float
    trajectory: List[Tuple[float, float]] = field(default_factory=list)


def run_episode(
    arena: Arena,
    config: PlannerConfig,
    start: Tuple[float, float, float],
    goal: Tuple[float, float],
    dt: float = 0.1,
    timeout_s: float = 90.0,
    robot_radius: float = BURGER_RADIUS,
    record: bool = False,
    planner_cls=AvoidancePlanner,
) -> EpisodeResult:
    """
    Drive the planner from start pose to goal; stop on success, collision or timeout.

    planner_cls lets the benchmark swap in the C++ planner (CppAvoidancePlanner).
    """
    planner = planner_cls(config)
    x, y, yaw = start
    gx, gy = goal
    straight = math.hypot(gx - x, gy - y)
    length = 0.0
    min_clear = arena.clearance(x, y) - robot_radius
    traj = [(x, y)] if record else []
    steps = int(timeout_s / dt)
    for step in range(steps):
        dist = math.hypot(gx - x, gy - y)
        ang = normalize_angle(math.atan2(gy - y, gx - x) - yaw)
        scan = raycast(arena, x, y, yaw)
        d = planner.plan(
            scan,
            angle_min=0.0,
            angle_increment=2.0 * math.pi / len(scan),
            range_min=0.12,
            range_max=3.5,
            goal_distance=dist,
            goal_angle_rad=ang,
        )
        if d.state == NavigationState.GOAL_REACHED:
            return EpisodeResult(
                config.algorithm, True, False, False, step * dt,
                length, straight, min_clear, traj,
            )
        nx = x + d.linear_x * math.cos(yaw) * dt
        ny = y + d.linear_x * math.sin(yaw) * dt
        yaw = normalize_angle(yaw + d.angular_z * dt)
        length += math.hypot(nx - x, ny - y)
        x, y = nx, ny
        if record:
            traj.append((x, y))
        clear = arena.clearance(x, y) - robot_radius
        min_clear = min(min_clear, clear)
        if clear <= 0.0:
            return EpisodeResult(
                config.algorithm, False, True, False, (step + 1) * dt,
                length, straight, min_clear, traj,
            )
    return EpisodeResult(
        config.algorithm, False, False, True, timeout_s,
        length, straight, min_clear, traj,
    )


def sample_free_point(
    arena: Arena, rng: random.Random, margin: float = 0.35
) -> Tuple[float, float]:
    """Sample a point at least ``margin`` metres from every obstacle."""
    xmin, ymin, xmax, ymax = arena.bounds
    for _ in range(10000):
        p = (rng.uniform(xmin, xmax), rng.uniform(ymin, ymax))
        if arena.clearance(*p) >= margin:
            return p
    raise RuntimeError('could not sample a free point; arena too cluttered')


def random_clutter(
    arena: Arena,
    rng: random.Random,
    count: int,
    keep_clear: Sequence[Tuple[float, float]] = (),
    radius_range: Tuple[float, float] = (0.10, 0.25),
) -> Arena:
    """Return a copy of ``arena`` with ``count`` extra random pillars added."""
    out = arena.copy()
    added = 0
    for _ in range(count * 200):
        if added >= count:
            break
        r = rng.uniform(*radius_range)
        x, y = sample_free_point(out, rng, margin=r + 0.30)
        if all(math.hypot(x - kx, y - ky) > r + 0.5 for kx, ky in keep_clear):
            out.circles.append(Circle(x, y, r))
            added += 1
    return out
