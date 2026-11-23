// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Anson Antony

#ifndef MY_ROBOT_NAVIGATION_CPP__WAYPOINTS_HPP_
#define MY_ROBOT_NAVIGATION_CPP__WAYPOINTS_HPP_

#include <string>
#include <utility>
#include <vector>

namespace my_robot_navigation_cpp
{

using Waypoint = std::pair<double, double>;

// Parse "x1,y1; x2,y2; ..." into waypoints. Throws std::invalid_argument on bad input.
std::vector<Waypoint> parse_waypoints(const std::string & text);

}  // namespace my_robot_navigation_cpp

#endif  // MY_ROBOT_NAVIGATION_CPP__WAYPOINTS_HPP_
