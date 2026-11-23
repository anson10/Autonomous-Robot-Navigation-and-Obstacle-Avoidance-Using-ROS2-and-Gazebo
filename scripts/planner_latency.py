#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Anson Antony
"""
Time one plan() call for the Python and C++ planners on identical 360-beam scans.

Scans come from 2D-simulator episodes in the arena. The C++ figure is measured
through the pybind11 module, so it includes converting the scan list to a
std::vector; the planner itself is a bit faster than shown.
Run inside a sourced workspace: python3 scripts/planner_latency.py
"""

import math
import statistics
import time

from my_robot_navigation.avoidance_planner import AvoidancePlanner, PlannerConfig
from my_robot_navigation.benchmark import default_world_path, make_scenarios
from my_robot_navigation.sim2d import load_world, raycast
from my_robot_navigation_cpp import _planner, CppAvoidancePlanner


def collect_inputs(n_scenarios: int = 8, steps: int = 150):
    base = load_world(str(default_world_path()))
    inputs = []
    for sc in make_scenarios(base, n_scenarios, seed=5, clutter=4, min_dist=2.5):
        planner = AvoidancePlanner(PlannerConfig())
        x, y, yaw = sc['start']
        gx, gy = sc['goal']
        for _ in range(steps):
            dist = math.hypot(gx - x, gy - y)
            ang = math.atan2(gy - y, gx - x) - yaw
            scan = raycast(sc['arena'], x, y, yaw)
            inputs.append((scan, dist, ang))
            d = planner.plan(scan, 0.0, 2 * math.pi / 360, 0.12, 3.5, False, dist, ang)
            if d.state.value == 'GOAL_REACHED':
                break
            x += d.linear_x * math.cos(yaw) * 0.1
            y += d.linear_x * math.sin(yaw) * 0.1
            yaw += d.angular_z * 0.1
    return inputs


def time_calls(call, inputs, repeats: int = 3):
    samples = []
    for _ in range(repeats):
        for scan, dist, ang in inputs:
            t0 = time.perf_counter_ns()
            call(scan, dist, ang)
            samples.append((time.perf_counter_ns() - t0) / 1000.0)
    samples.sort()
    return statistics.median(samples), samples[int(0.99 * (len(samples) - 1))]


def main() -> None:
    inputs = collect_inputs()
    inc = 2 * math.pi / 360
    print(f'{len(inputs)} scans x 3 repeats, 360 beams each\n')
    print('| Planner | Python median (us) | C++ median (us) | Python p99 (us) '
          '| C++ p99 (us) | Speed-up (median) |')
    print('| :--- | ---: | ---: | ---: | ---: | ---: |')
    for algo in ('sector', 'apf', 'follow_the_gap'):
        py = AvoidancePlanner(PlannerConfig(algorithm=algo))
        cpp_cfg = CppAvoidancePlanner._to_cpp(PlannerConfig(algorithm=algo))
        cpp = _planner.AvoidancePlanner(cpp_cfg)
        py_med, py_p99 = time_calls(
            lambda s, d, a: py.plan(s, 0.0, inc, 0.12, 3.5, False, d, a), inputs)
        cpp_med, cpp_p99 = time_calls(
            lambda s, d, a: cpp.plan(s, 0.0, inc, 0.12, 3.5, False, d, a), inputs)
        print(f'| `{algo}` | {py_med:.0f} | {cpp_med:.0f} | {py_p99:.0f} | {cpp_p99:.0f} '
              f'| {py_med / cpp_med:.0f}x |')


if __name__ == '__main__':
    main()
