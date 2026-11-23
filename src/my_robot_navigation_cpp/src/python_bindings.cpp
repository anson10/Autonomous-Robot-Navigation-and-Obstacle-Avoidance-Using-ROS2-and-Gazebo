// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Anson Antony

// pybind11 module exposing the C++ planner to Python, so the 2D benchmark and the
// parity test can run the exact code the C++ node uses.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <optional>
#include <string>
#include <vector>

#include "my_robot_navigation_cpp/avoidance_planner.hpp"

namespace py = pybind11;
using my_robot_navigation_cpp::AvoidanceDecision;
using my_robot_navigation_cpp::AvoidancePlanner;
using my_robot_navigation_cpp::kNumSectors;
using my_robot_navigation_cpp::kSectorNames;
using my_robot_navigation_cpp::PlannerConfig;
using my_robot_navigation_cpp::ScanInput;

PYBIND11_MODULE(_planner, m)
{
  m.doc() = "C++ obstacle avoidance planner";

  py::class_<PlannerConfig>(m, "PlannerConfig")
  .def(py::init<>())
  .def_readwrite("algorithm", &PlannerConfig::algorithm)
  .def_readwrite("safe_distance", &PlannerConfig::safe_distance)
  .def_readwrite("critical_distance", &PlannerConfig::critical_distance)
  .def_readwrite("max_linear_speed", &PlannerConfig::max_linear_speed)
  .def_readwrite("min_linear_speed", &PlannerConfig::min_linear_speed)
  .def_readwrite("recovery_linear_speed", &PlannerConfig::recovery_linear_speed)
  .def_readwrite("max_angular_speed", &PlannerConfig::max_angular_speed)
  .def_readwrite("front_fov_deg", &PlannerConfig::front_fov_deg)
  .def_readwrite("apf_attractive_gain", &PlannerConfig::apf_attractive_gain)
  .def_readwrite("apf_repulsive_gain", &PlannerConfig::apf_repulsive_gain)
  .def_readwrite("apf_influence_distance", &PlannerConfig::apf_influence_distance)
  .def_readwrite("gap_threshold_distance", &PlannerConfig::gap_threshold_distance)
  .def_readwrite("stuck_timeout_steps", &PlannerConfig::stuck_timeout_steps)
  .def_readwrite("recovery_hold_steps", &PlannerConfig::recovery_hold_steps)
  .def_readwrite("trap_timeout_steps", &PlannerConfig::trap_timeout_steps)
  .def_readwrite("wall_follow_hold_steps", &PlannerConfig::wall_follow_hold_steps)
  .def_readwrite("robot_radius", &PlannerConfig::robot_radius)
  .def_readwrite("goal_tolerance", &PlannerConfig::goal_tolerance);

  py::class_<AvoidancePlanner>(m, "AvoidancePlanner")
  .def(py::init<PlannerConfig>(), py::arg("config") = PlannerConfig())
  .def("reset_state", &AvoidancePlanner::reset_state)
  .def_property(
    "config", [](AvoidancePlanner & p) {return p.config();},
    [](AvoidancePlanner & p, const PlannerConfig & c) {p.config() = c;})
  .def(
    "plan",
    [](AvoidancePlanner & planner, const std::vector<double> & ranges, double angle_min,
    double angle_increment, double range_min, double range_max, bool paused,
    std::optional<double> goal_distance, double goal_angle_rad) {
      const ScanInput scan{ranges, angle_min, angle_increment, range_min, range_max};
      const AvoidanceDecision d = planner.plan(scan, paused, goal_distance, goal_angle_rad);
      py::dict sectors;
      if (d.has_sectors) {
        for (int s = 0; s < kNumSectors; ++s) {
          sectors[kSectorNames[s]] = d.sector_distances[s];
        }
      }
      return py::make_tuple(
        d.linear_x, d.angular_z, my_robot_navigation_cpp::to_string(d.state),
        d.min_front_distance, d.target_heading_rad, sectors);
    },
    py::arg("ranges"), py::arg("angle_min") = 0.0, py::arg("angle_increment") = 0.0,
    py::arg("range_min") = 0.05, py::arg("range_max") = 3.5, py::arg("paused") = false,
    py::arg("goal_distance") = py::none(), py::arg("goal_angle_rad") = 0.0);
}
