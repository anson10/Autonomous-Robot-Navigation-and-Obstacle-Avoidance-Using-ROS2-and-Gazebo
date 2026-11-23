#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Anson Antony

"""Interactive 2D dashboard (and GIF/PNG exporter) for the AvoidancePlanner.

The arena comes from the same Gazebo world file the ROS launch uses, and the
planner runs with the ROS node's default parameters (0.22 m/s, 1.2 rad/s), so
what you see here matches what the node is asked to do in Gazebo. Physics is a
simple unicycle model: no inertia, slip or sensor noise.

Controls (interactive mode): left-click sets a goal, right-click adds/removes
a pillar, space pauses, R resets.
"""

import argparse
import math
from pathlib import Path
import sys
from typing import List, Optional

import matplotlib.patches as patches
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, RadioButtons, Slider

REPO_ROOT = Path(__file__).resolve().parent
PKG_ROOT = REPO_ROOT / 'src' / 'my_robot_navigation'
sys.path.insert(0, str(PKG_ROOT))

from my_robot_navigation.avoidance_planner import (  # noqa: E402
    AvoidancePlanner,
    NavigationState,
    normalize_angle,
    PlannerConfig,
)
from my_robot_navigation.sim2d import (  # noqa: E402
    BURGER_RADIUS,
    Circle,
    load_world,
    raycast,
)

WORLD = PKG_ROOT / 'worlds' / 'obstacle_arena.world'
START_POSE = (-2.2, 0.5, -0.1)  # same spawn pose as the Gazebo launch file
TOUR = [(0.0, 0.62), (0.68, 0.0), (0.0, -0.62), (-0.68, 0.0),
        (-1.8, 1.8), (1.8, 1.8), (1.8, -1.8), (-1.8, -1.8)]

STATE_COLORS = {
    NavigationState.CLEAR: '#2ecc71',
    NavigationState.AVOIDING: '#f39c12',
    NavigationState.WALL_FOLLOW: '#00bcd4',
    NavigationState.CRITICAL_STOP: '#e74c3c',
    NavigationState.RECOVERY: '#9b59b6',
    NavigationState.PAUSED: '#95a5a6',
    NavigationState.GOAL_REACHED: '#3498db',
}
SECTOR_WEDGES = {
    'right': (-90.0, -50.0), 'front_right': (-50.0, -15.0), 'front': (-15.0, 15.0),
    'front_left': (15.0, 50.0), 'left': (50.0, 90.0),
}
BG, PANEL, GRID, TEXT, ACCENT = '#14171c', '#1c2027', '#2a2f38', '#d7dae0', '#61afef'


def clearance_rgb(d: float, critical: float, safe: float):
    """Red at the critical distance, through yellow, to green at 2x safe."""
    t = max(0.0, min(1.0, (d - critical) / max(2 * safe - critical, 1e-3)))
    return (min(1.0, 2 * (1 - t)), min(1.0, 2 * t), 0.2)


def run(algorithm: str, frames: int, save: Optional[str], tour: bool, dt: float) -> None:
    """Run the dashboard live, or render ``frames`` frames to a GIF/PNG."""
    interactive = save is None
    base_arena = load_world(str(WORLD))
    arena = base_arena.copy()
    planner = AvoidancePlanner(PlannerConfig(algorithm=algorithm))
    cfg = planner.config

    sim = {'x': START_POSE[0], 'y': START_POSE[1], 'yaw': START_POSE[2],
           'wp': 0, 'goal': TOUR[0] if tour else (1.8, -1.8), 'tour': tour,
           'paused': False, 'time': 0.0, 'dist': 0.0, 'reached': 0}
    trail_x: List[float] = [sim['x']]
    trail_y: List[float] = [sim['y']]

    fig = plt.figure(figsize=(12.0, 7.4) if interactive else (10.0, 7.0), facecolor=BG)
    ax = fig.add_axes([0.03, 0.05, 0.60, 0.88] if interactive else [0.02, 0.04, 0.62, 0.90],
                      facecolor=PANEL)
    xmin, ymin, xmax, ymax = arena.bounds
    ax.set_xlim(xmin - 0.1, xmax + 0.1)
    ax.set_ylim(ymin - 0.1, ymax + 0.1)
    ax.set_aspect('equal')
    ax.grid(True, color=GRID, linestyle='--', linewidth=0.5)
    ax.tick_params(colors='#7f8794', labelsize=8)
    for spine in ax.spines.values():
        spine.set_color(GRID)

    for b in arena.boxes:
        ax.add_patch(patches.Rectangle((b.xmin, b.ymin), b.xmax - b.xmin, b.ymax - b.ymin,
                                       facecolor='#3b4252', edgecolor='#4c566a', zorder=3))
    pillar_patches: List[patches.Circle] = []

    def redraw_pillars() -> None:
        for p in pillar_patches:
            p.remove()
        pillar_patches.clear()
        for c in arena.circles:
            patch = patches.Circle((c.x, c.y), c.r, facecolor='#be5046',
                                   edgecolor='#e06c75', alpha=0.9, zorder=3)
            ax.add_patch(patch)
            pillar_patches.append(patch)

    redraw_pillars()

    wedge_patches = {}
    for name in SECTOR_WEDGES:
        w = patches.Wedge((0, 0), 1.0, 0, 1, alpha=0.22, zorder=2, linewidth=0)
        ax.add_patch(w)
        wedge_patches[name] = w
    safety = patches.Circle((0, 0), cfg.safe_distance, fill=False, linestyle='--',
                            linewidth=1.2, zorder=4)
    ax.add_patch(safety)
    (trail_line,) = ax.plot([], [], '-', color=ACCENT, lw=2.0, alpha=0.9, zorder=4)
    hits = ax.scatter([], [], s=6, c='#ff6b6b', alpha=0.9, zorder=4)
    (goal_line,) = ax.plot([], [], ':', color='#e5c07b', lw=1.2, zorder=4)
    (goal_mark,) = ax.plot([], [], marker='*', ms=16, color='#e5c07b',
                           markeredgecolor='black', zorder=7, ls='none')
    (tour_marks,) = ax.plot([], [], marker='o', ms=5, color='#e5c07b', alpha=0.35,
                            zorder=3, ls='none')
    robot = patches.Circle((0, 0), BURGER_RADIUS, facecolor=ACCENT, edgecolor='white',
                           zorder=6)
    ax.add_patch(robot)
    (heading,) = ax.plot([], [], '-', color='white', lw=2.5, zorder=7)
    title = ax.set_title('', color='white', fontsize=11, fontweight='bold', pad=8)

    # Side panel: HUD + legend (both modes), widgets (interactive only).
    hud = fig.text(0.66, 0.90 if not interactive else 0.45, '', color=TEXT,
                   fontsize=9.5 if not interactive else 8.8,
                   family='monospace', va='top',
                   bbox={'facecolor': PANEL, 'edgecolor': GRID, 'pad': 9})
    legend_y, legend_step = (0.48, 0.04) if not interactive else (0.19, 0.034)
    for i, (state, color) in enumerate(
        [(s, c) for s, c in STATE_COLORS.items() if s != NavigationState.PAUSED]
    ):
        fig.patches.append(patches.Rectangle((0.665, legend_y - i * legend_step), 0.015, 0.022,
                                             transform=fig.transFigure, color=color))
        fig.text(0.69, legend_y - i * legend_step + 0.003, state.value, color=TEXT, fontsize=8.5)
    if not interactive:
        fig.text(0.66, 0.18, 'Sector wedges: clearance per LiDAR sector\n'
                 '(red = critical, green = open).\nDashed ring: safe_distance.',
                 color='#9aa1ab', fontsize=8.2, va='top')

    widgets = {}
    if interactive:
        fig.text(0.66, 0.93, 'Left-click: set goal   Right-click: add/remove pillar\n'
                 'Space: pause   R: reset', color='#9aa1ab', fontsize=8.5, va='top')
        ax_radio = fig.add_axes([0.66, 0.70, 0.30, 0.14], facecolor=PANEL)
        algos = ('sector', 'apf', 'follow_the_gap')
        radio = RadioButtons(ax_radio, algos, active=algos.index(algorithm))
        for lbl in radio.labels:
            lbl.set_color('white')
        radio.on_clicked(lambda label: setattr(cfg, 'algorithm', label))
        widgets['radio'] = radio

        s_safe = Slider(fig.add_axes([0.72, 0.63, 0.22, 0.03], facecolor=PANEL),
                        'safe', 0.35, 1.0, valinit=cfg.safe_distance, color='#2ecc71')
        s_speed = Slider(fig.add_axes([0.72, 0.58, 0.22, 0.03], facecolor=PANEL),
                         'speed', 0.10, 0.40, valinit=cfg.max_linear_speed, color=ACCENT)
        for s in (s_safe, s_speed):
            s.label.set_color('white')
            s.valtext.set_color('white')
        s_safe.on_changed(lambda v: setattr(cfg, 'safe_distance', float(v)))
        s_speed.on_changed(lambda v: setattr(cfg, 'max_linear_speed', float(v)))
        widgets['sliders'] = (s_safe, s_speed)

        def reset(_event=None) -> None:
            sim.update({'x': START_POSE[0], 'y': START_POSE[1], 'yaw': START_POSE[2],
                        'wp': 0, 'tour': True, 'goal': TOUR[0], 'time': 0.0,
                        'dist': 0.0, 'reached': 0})
            arena.circles[:] = list(base_arena.circles)
            trail_x[:], trail_y[:] = [sim['x']], [sim['y']]
            planner.reset_state()
            redraw_pillars()

        def toggle_pause(_event=None) -> None:
            sim['paused'] = not sim['paused']

        def resume_tour(_event=None) -> None:
            sim['tour'] = True
            sim['goal'] = TOUR[sim['wp'] % len(TOUR)]

        for i, (label, cb) in enumerate(
            [('Pause / resume', toggle_pause), ('Waypoint tour', resume_tour),
             ('Reset', reset)]
        ):
            btn = Button(fig.add_axes([0.66 + i * 0.105, 0.50, 0.10, 0.045]), label,
                         color='#2c323d', hovercolor='#3e4451')
            btn.label.set_color('white')
            btn.label.set_fontsize(8.5)
            btn.on_clicked(cb)
            widgets[label] = btn

        def on_click(event) -> None:
            if event.inaxes != ax or event.xdata is None:
                return
            cx, cy = float(event.xdata), float(event.ydata)
            if event.button == 1:
                sim['goal'], sim['tour'] = (cx, cy), False
                planner.reset_state()
            elif event.button == 3:
                for i, c in enumerate(arena.circles):
                    if math.hypot(cx - c.x, cy - c.y) <= c.r + 0.08:
                        arena.circles.pop(i)
                        break
                else:
                    arena.circles.append(Circle(cx, cy, 0.2))
                redraw_pillars()

        def on_key(event) -> None:
            if event.key == ' ':
                toggle_pause()
            elif event.key in ('r', 'R'):
                reset()

        fig.canvas.mpl_connect('button_press_event', on_click)
        fig.canvas.mpl_connect('key_press_event', on_key)

    def update(_frame: int):
        x, y, yaw = sim['x'], sim['y'], sim['yaw']
        gx, gy = sim['goal']
        dist = math.hypot(gx - x, gy - y)
        if dist <= cfg.goal_tolerance and sim['tour']:
            sim['wp'] = (sim['wp'] + 1) % len(TOUR)
            sim['reached'] += 1
            sim['goal'] = TOUR[sim['wp']]
            gx, gy = sim['goal']
            dist = math.hypot(gx - x, gy - y)
        bearing = normalize_angle(math.atan2(gy - y, gx - x) - yaw)

        scan = raycast(arena, x, y, yaw)
        d = planner.plan(scan, angle_min=0.0, angle_increment=2 * math.pi / len(scan),
                         range_min=0.12, range_max=3.5, paused=sim['paused'],
                         goal_distance=dist, goal_angle_rad=bearing)
        if not sim['paused']:
            nx = x + d.linear_x * math.cos(yaw) * dt
            ny = y + d.linear_x * math.sin(yaw) * dt
            sim['dist'] += math.hypot(nx - x, ny - y)
            x, y = nx, ny
            yaw = normalize_angle(yaw + d.angular_z * dt)
            sim.update({'x': x, 'y': y, 'yaw': yaw, 'time': sim['time'] + dt})
            trail_x.append(x)
            trail_y.append(y)
            if len(trail_x) > 1500:
                del trail_x[0], trail_y[0]

        color = STATE_COLORS.get(d.state, '#2ecc71')
        robot.center = safety.center = (x, y)
        safety.set_radius(cfg.safe_distance)
        safety.set_edgecolor(color)
        for name, (lo, hi) in SECTOR_WEDGES.items():
            r = min(d.sector_distances.get(name, 3.5), 3.5)
            w = wedge_patches[name]
            w.set_center((x, y))
            w.set_radius(r)
            w.set_theta1(math.degrees(yaw) + lo)
            w.set_theta2(math.degrees(yaw) + hi)
            w.set_facecolor(clearance_rgb(r, cfg.critical_distance, cfg.safe_distance))
        trail_line.set_data(trail_x, trail_y)
        goal_mark.set_data([gx], [gy])
        goal_line.set_data([x, gx], [y, gy])
        if sim['tour']:
            tour_marks.set_data([p[0] for p in TOUR], [p[1] for p in TOUR])
        else:
            tour_marks.set_data([], [])
        tw = yaw + d.target_heading_rad
        heading.set_data([x, x + 0.45 * math.cos(tw)], [y, y + 0.45 * math.sin(tw)])
        heading.set_color(color)
        pts = [(x + r * math.cos(yaw + 2 * math.pi * i / len(scan)),
                y + r * math.sin(yaw + 2 * math.pi * i / len(scan)))
               for i, r in enumerate(scan) if math.isfinite(r) and i % 2 == 0]
        hits.set_offsets(pts if pts else [[float('nan'), float('nan')]])

        title.set_text(f'{cfg.algorithm}  |  {d.state.value}')
        title.set_color(color)
        hud.set_text(
            f'algorithm   {cfg.algorithm}\n'
            f'state       {d.state.value}\n'
            f'v / w       {d.linear_x:4.2f} m/s  {d.angular_z:+5.2f} rad/s\n'
            f'front       {d.min_front_distance:4.2f} m\n'
            f'goal dist   {dist:4.2f} m\n'
            f'sim time    {sim["time"]:5.1f} s\n'
            f'driven      {sim["dist"]:5.2f} m\n'
            f'goals hit   {sim["reached"]}'
        )
        return ()

    if save and Path(save).suffix.lower() == '.png':
        for i in range(frames):
            update(i)
        fig.savefig(save, dpi=130, facecolor=fig.get_facecolor())
        print(f'Saved {save}')
        return
    anim = FuncAnimation(fig, update, frames=frames, interval=40, blit=False)
    if save:
        anim.save(save, writer='pillow', fps=20, dpi=72)
        print(f'Saved {save}')
    else:
        plt.show()


def main() -> None:
    """Parse arguments and start the dashboard."""
    parser = argparse.ArgumentParser(description='2D obstacle avoidance dashboard.')
    parser.add_argument('--algorithm', choices=['sector', 'apf', 'follow_the_gap'],
                        default='sector')
    parser.add_argument('--frames', type=int, default=600,
                        help='frames to render when saving (default 600)')
    parser.add_argument('--dt', type=float, default=0.15,
                        help='simulated seconds per frame (default 0.15)')
    parser.add_argument('--no-tour', action='store_true',
                        help='single goal instead of the waypoint tour')
    parser.add_argument('--save', default=None, help='write a .gif or .png instead')
    args = parser.parse_args()
    run(args.algorithm, args.frames, args.save, not args.no_tour, args.dt)


if __name__ == '__main__':
    main()
