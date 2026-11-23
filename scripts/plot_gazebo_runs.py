#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Anson Antony
"""Plot Gazebo tour trajectories and print a results table from results/gazebo/*/."""

import csv
import json
import math
from pathlib import Path
import sys

import matplotlib.patches as patches
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'my_robot_navigation'))
from my_robot_navigation.sim2d import BURGER_RADIUS, load_world  # noqa: E402

WAYPOINTS = [(0.0, 0.62), (0.68, 0.0), (0.0, -0.62), (-0.68, 0.0),
             (-1.8, 1.8), (1.8, 1.8), (1.8, -1.8), (-1.8, -1.8)]
GOAL_TOLERANCE = 0.22  # matches goal_tolerance in navigation_params.yaml
COLORS = {'sector': '#61afef', 'apf': '#e5c07b', 'follow_the_gap': '#c678dd'}


def analyse(run_dir: Path, arena):
    odom = sorted((float(r['t']), float(r['x']), float(r['y']))
                  for r in csv.DictReader(open(run_dir / 'odom.csv')))
    status = [json.loads(line) for line in open(run_dir / 'status.jsonl')]
    reached = max(s['goals_reached'] for s in status)  # as counted by the node
    # Arrival times from odometry (sim time): waypoint i counts once the robot is
    # within the node's goal tolerance of it after reaching waypoint i - 1.
    t0, idx, done_t = odom[0][0], 0, None
    for t, x, y in odom:
        if idx < len(WAYPOINTS) and math.hypot(
            x - WAYPOINTS[idx][0], y - WAYPOINTS[idx][1]
        ) <= GOAL_TOLERANCE:
            idx += 1
            if idx == len(WAYPOINTS):
                done_t = t - t0
                break
    path = [p for p in odom if done_t is None or p[0] <= t0 + done_t]
    dist = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(path, path[1:]))
    clear = min(arena.clearance(x, y) - BURGER_RADIUS for _, x, y in path)
    return {'path': path, 'reached': reached, 'odom_reached': idx, 'time': done_t,
            'dist': dist, 'clear': clear}


def main() -> None:
    arena = load_world(str(ROOT / 'src/my_robot_navigation/worlds/obstacle_arena.world'))
    order = ['sector', 'apf', 'follow_the_gap']
    dirs = [d for d in (ROOT / 'results/gazebo').iterdir() if (d / 'odom.csv').exists()]
    dirs.sort(key=lambda d: order.index(d.name) if d.name in order else len(order))
    runs = {d.name: analyse(d, arena) for d in dirs}
    fig, axes = plt.subplots(1, len(runs), figsize=(5.2 * len(runs), 5.4), facecolor='white')
    axes = [axes] if len(runs) == 1 else axes
    for ax, (name, r) in zip(axes, runs.items()):
        for b in arena.boxes:
            ax.add_patch(patches.Rectangle((b.xmin, b.ymin), b.xmax - b.xmin, b.ymax - b.ymin,
                                           color='#4c566a'))
        for c in arena.circles:
            ax.add_patch(patches.Circle((c.x, c.y), c.r, color='#be5046'))
        xs, ys = [p[1] for p in r['path']], [p[2] for p in r['path']]
        ax.plot(xs, ys, color=COLORS.get(name, 'k'), lw=2)
        ax.plot(xs[0], ys[0], 'o', color='black', ms=6)
        for i, (wx, wy) in enumerate(WAYPOINTS, 1):
            ax.plot(wx, wy, '*', color='#d19a66', ms=11, markeredgecolor='black')
            ax.annotate(str(i), (wx, wy), xytext=(4, 4), textcoords='offset points', fontsize=8)
        t = f"{r['time']:.0f} s" if r['time'] is not None else 'not finished'
        ax.set_title(f"{name}: {r['reached']}/8 goals, {t}", fontsize=11)
        ax.set_aspect('equal')
        ax.set_xlim(-3.15, 3.15)
        ax.set_ylim(-3.15, 3.15)
        ax.grid(alpha=0.3)
    fig.suptitle('Gazebo (TurtleBot3 Burger): 8-waypoint tour, trajectories from /odom',
                 fontsize=12)
    fig.tight_layout()
    out = ROOT / 'images' / 'gazebo_tour.png'
    fig.savefig(out, dpi=110)
    print(f'Saved {out}\n')
    print('| Algorithm | Goals reached | Tour time (sim s) | Distance driven (m) '
          '| Min clearance (m) |')
    print('| :--- | ---: | ---: | ---: | ---: |')
    for name, r in runs.items():
        t = f"{r['time']:.0f}" if r['time'] is not None else '-'
        print(f"| `{name}` | {r['reached']}/8 | {t} | {r['dist']:.1f} | {r['clear']:.2f} |")


if __name__ == '__main__':
    main()
