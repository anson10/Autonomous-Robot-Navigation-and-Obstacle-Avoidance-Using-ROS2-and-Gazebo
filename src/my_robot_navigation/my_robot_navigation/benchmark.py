# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Anson Antony

"""
Seeded benchmark of the three avoidance algorithms in the 2D simulator.

Every algorithm runs on the *same* scenarios: the Gazebo arena plus a few random
extra pillars, with random start and goal points. Results are reported per
algorithm as success / collision / timeout rates, time to goal, path efficiency
(straight-line distance divided by distance driven) and minimum clearance.

Run with ``ros2 run my_robot_navigation benchmark --trials 50 --seed 7``.
"""

import argparse
from concurrent.futures import ProcessPoolExecutor
import csv
import math
from pathlib import Path
import random
import statistics
import sys
from typing import Dict, List, Optional, Sequence, Tuple

from my_robot_navigation.avoidance_planner import AvoidancePlanner, PlannerConfig
from my_robot_navigation.sim2d import (
    Arena,
    EpisodeResult,
    load_world,
    random_clutter,
    run_episode,
    sample_free_point,
)

ALGORITHMS = ('sector', 'apf', 'follow_the_gap')


def default_world_path() -> Path:
    """Locate obstacle_arena.world in the installed share dir or source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory('my_robot_navigation'))
        candidate = share / 'worlds' / 'obstacle_arena.world'
        if candidate.exists():
            return candidate
    except Exception:  # noqa: B902 - ament_index missing or package not built
        pass
    return Path(__file__).resolve().parents[1] / 'worlds' / 'obstacle_arena.world'


def make_scenarios(
    base: Arena, trials: int, seed: int, clutter: int, min_dist: float
) -> List[Dict]:
    """Build reproducible (arena, start, goal) scenarios."""
    rng = random.Random(seed)
    scenarios = []
    while len(scenarios) < trials:
        start_xy = sample_free_point(base, rng, margin=0.40)
        goal = sample_free_point(base, rng, margin=0.40)
        if math.hypot(goal[0] - start_xy[0], goal[1] - start_xy[1]) < min_dist:
            continue
        arena = random_clutter(base, rng, clutter, keep_clear=(start_xy, goal))
        yaw = rng.uniform(-math.pi, math.pi)
        scenarios.append(
            {'arena': arena, 'start': (*start_xy, yaw), 'goal': goal}
        )
    return scenarios


def summarize(results: Sequence[EpisodeResult]) -> Dict[str, float]:
    """Aggregate episode results into summary metrics."""
    n = len(results)
    ok = [r for r in results if r.success]

    def mean(vals: List[float]) -> float:
        return statistics.fmean(vals) if vals else float('nan')

    return {
        'trials': n,
        'success_rate': len(ok) / n,
        'collision_rate': sum(r.collided for r in results) / n,
        'timeout_rate': sum(r.timed_out for r in results) / n,
        'mean_time_s': mean([r.sim_time_s for r in ok]),
        'path_efficiency': mean(
            [r.straight_line_m / r.path_length_m for r in ok if r.path_length_m > 0]
        ),
        'mean_min_clearance_m': mean([r.min_clearance_m for r in ok]),
    }


def format_markdown(summaries: Dict[str, Dict[str, float]]) -> str:
    """Render summaries as a Markdown table."""
    lines = [
        '| Algorithm | Success | Collision | Timeout | Time to goal (s) '
        '| Path efficiency | Min clearance (m) |',
        '| :--- | ---: | ---: | ---: | ---: | ---: | ---: |',
    ]

    def num(value: float, fmt: str) -> str:
        return '-' if math.isnan(value) else format(value, fmt)

    for algo, s in summaries.items():
        lines.append(
            f"| `{algo}` | {s['success_rate']:.0%} | {s['collision_rate']:.0%} "
            f"| {s['timeout_rate']:.0%} | {num(s['mean_time_s'], '.1f')} "
            f"| {num(s['path_efficiency'], '.2f')} "
            f"| {num(s['mean_min_clearance_m'], '.2f')} |"
        )
    return '\n'.join(lines)


def planner_class(impl: str):
    """Return the planner class for 'python' or 'cpp' (needs my_robot_navigation_cpp)."""
    if impl == 'cpp':
        from my_robot_navigation_cpp import CppAvoidancePlanner
        return CppAvoidancePlanner
    return AvoidancePlanner


def _run_one(job: Tuple[str, Dict, float, str]) -> EpisodeResult:
    algo, sc, timeout, impl = job
    return run_episode(
        sc['arena'], PlannerConfig(algorithm=algo), sc['start'], sc['goal'],
        timeout_s=timeout, planner_cls=planner_class(impl),
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run the benchmark from the command line."""
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--world', type=Path, default=None)
    parser.add_argument('--trials', type=int, default=30)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument(
        '--clutter', type=int, default=4, help='extra random pillars per trial'
    )
    parser.add_argument('--min-dist', type=float, default=2.5)
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument(
        '--algorithms', nargs='+', choices=ALGORITHMS, default=list(ALGORITHMS)
    )
    parser.add_argument('--csv', type=Path, default=None)
    parser.add_argument(
        '--jobs', type=int, default=1, help='parallel worker processes'
    )
    parser.add_argument(
        '--impl', choices=['python', 'cpp'], default='python',
        help='planner implementation to benchmark',
    )
    args = parser.parse_args(argv)

    world = args.world or default_world_path()
    base = load_world(str(world))
    scenarios = make_scenarios(
        base, args.trials, args.seed, args.clutter, args.min_dist
    )

    rows = []
    summaries: Dict[str, Dict[str, float]] = {}
    planner_class(args.impl)  # fail early if the C++ module is missing
    jobs = [(a, sc, args.timeout, args.impl) for a in args.algorithms for sc in scenarios]
    if args.jobs > 1:
        with ProcessPoolExecutor(args.jobs) as pool:
            all_results = list(pool.map(_run_one, jobs, chunksize=4))
    else:
        all_results = [_run_one(job) for job in jobs]

    for k, algo in enumerate(args.algorithms):
        results = all_results[k * len(scenarios):(k + 1) * len(scenarios)]
        for i, res in enumerate(results):
            rows.append({
                'algorithm': algo,
                'trial': i,
                'success': res.success,
                'collided': res.collided,
                'timed_out': res.timed_out,
                'sim_time_s': round(res.sim_time_s, 2),
                'path_length_m': round(res.path_length_m, 3),
                'straight_line_m': round(res.straight_line_m, 3),
                'min_clearance_m': round(res.min_clearance_m, 3),
            })
        summaries[algo] = summarize(results)

    if args.csv:
        with open(args.csv, 'w', newline='') as fh:
            writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

    print(
        f'impl={args.impl} world={world.name} trials={args.trials} seed={args.seed} '
        f'clutter={args.clutter} min_dist={args.min_dist}m '
        f'timeout={args.timeout}s'
    )
    print(format_markdown(summaries))
    return 0


if __name__ == '__main__':
    sys.exit(main())
