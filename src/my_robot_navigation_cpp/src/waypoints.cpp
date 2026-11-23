// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Anson Antony

#include "my_robot_navigation_cpp/waypoints.hpp"

#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace my_robot_navigation_cpp
{

std::vector<Waypoint> parse_waypoints(const std::string & text)
{
  std::vector<Waypoint> points;
  std::string normalized = text;
  for (char & c : normalized) {
    if (c == '\n') {
      c = ';';
    }
  }
  std::stringstream chunks(normalized);
  std::string chunk;
  while (std::getline(chunks, chunk, ';')) {
    if (chunk.find_first_not_of(" \t") == std::string::npos) {
      continue;
    }
    std::vector<std::string> parts;
    std::stringstream fields(chunk);
    std::string field;
    while (std::getline(fields, field, ',')) {
      parts.push_back(field);
    }
    if (parts.size() != 2) {
      throw std::invalid_argument("waypoint \"" + chunk + "\" must be \"x,y\"");
    }
    std::size_t used_x = 0;
    std::size_t used_y = 0;
    const double x = std::stod(parts[0], &used_x);
    const double y = std::stod(parts[1], &used_y);
    if (parts[0].find_first_not_of(" \t", used_x) != std::string::npos ||
      parts[1].find_first_not_of(" \t", used_y) != std::string::npos)
    {
      throw std::invalid_argument("waypoint \"" + chunk + "\" is not numeric");
    }
    points.emplace_back(x, y);
  }
  return points;
}

}  // namespace my_robot_navigation_cpp
