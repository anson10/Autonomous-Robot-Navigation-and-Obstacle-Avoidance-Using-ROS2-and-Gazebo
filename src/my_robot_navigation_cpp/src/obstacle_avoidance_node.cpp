// SPDX-License-Identifier: MIT
// Copyright (c) 2025-2026 Anson Antony

// ROS 2 obstacle avoidance node (C++). Same topics, parameters and services as the
// Python node in my_robot_navigation, so either can be launched with impl:=cpp|python.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_navigation_cpp/avoidance_planner.hpp"
#include "my_robot_navigation_cpp/waypoints.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace my_robot_navigation_cpp
{

using geometry_msgs::msg::Point;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;

namespace
{

constexpr double kPi = 3.14159265358979323846;

const std::vector<std::string> kAlgorithms{"sector", "apf", "follow_the_gap"};

// Sector wedges drawn in RViz: index into SectorDistances -> (start_deg, end_deg).
constexpr std::array<std::pair<int, std::pair<double, double>>, 5> kWedges{{
  {kRight, {-90.0, -50.0}}, {kFrontRight, {-50.0, -15.0}}, {kFront, {-15.0, 15.0}},
  {kFrontLeft, {15.0, 50.0}}, {kLeft, {50.0, 90.0}},
}};

ColorRGBA rgba(double r, double g, double b, double a)
{
  ColorRGBA c;
  c.r = static_cast<float>(r);
  c.g = static_cast<float>(g);
  c.b = static_cast<float>(b);
  c.a = static_cast<float>(a);
  return c;
}

// Red at/below critical distance, through yellow, to green at 2x safe distance.
ColorRGBA clearance_color(double distance, double critical, double safe)
{
  double t = (distance - critical) / std::max(2.0 * safe - critical, 1e-3);
  t = std::max(0.0, std::min(1.0, t));
  return rgba(std::min(1.0, 2.0 * (1.0 - t)), std::min(1.0, 2.0 * t), 0.15, 0.35);
}

ColorRGBA state_color(NavigationState state, double alpha)
{
  switch (state) {
    case NavigationState::kClear: return rgba(0.18, 0.80, 0.44, alpha);
    case NavigationState::kAvoiding: return rgba(0.95, 0.61, 0.07, alpha);
    case NavigationState::kWallFollow: return rgba(0.00, 0.74, 0.83, alpha);
    case NavigationState::kCriticalStop: return rgba(0.91, 0.30, 0.24, alpha);
    case NavigationState::kRecovery: return rgba(0.61, 0.35, 0.71, alpha);
    case NavigationState::kGoalReached: return rgba(0.20, 0.60, 0.86, alpha);
    default: return rgba(0.58, 0.65, 0.65, alpha);
  }
}

Point point(double x, double y, double z)
{
  Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

std::string json_number(double value, int decimals)
{
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(decimals);
  out << value;
  return out.str();
}

}  // namespace

class ObstacleAvoidanceNode : public rclcpp::Node
{
public:
  explicit ObstacleAvoidanceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("obstacle_avoidance_node", options)
  {
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "/scan", read_only);
    const auto cmd_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel", read_only);
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom", read_only);
    const auto goal_topic = declare_parameter<std::string>("goal_topic", "/goal_pose", read_only);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    publish_markers_ = declare_parameter<bool>("publish_markers", true);
    scan_timeout_s_ = declare_parameter<double>("scan_timeout_s", 0.5);
    loop_waypoints_ = declare_parameter<bool>("loop_waypoints", false);
    const auto waypoints = declare_parameter<std::string>("waypoints", "");

    PlannerConfig cfg;
    cfg.algorithm = declare_parameter<std::string>("algorithm", cfg.algorithm);
    cfg.safe_distance = declare_parameter<double>("safe_distance", cfg.safe_distance);
    cfg.critical_distance = declare_parameter<double>("critical_distance", cfg.critical_distance);
    cfg.max_linear_speed = declare_parameter<double>("max_linear_speed", cfg.max_linear_speed);
    cfg.min_linear_speed = declare_parameter<double>("min_linear_speed", cfg.min_linear_speed);
    cfg.recovery_linear_speed =
      declare_parameter<double>("recovery_linear_speed", cfg.recovery_linear_speed);
    cfg.max_angular_speed = declare_parameter<double>("max_angular_speed", cfg.max_angular_speed);
    cfg.front_fov_deg = declare_parameter<double>("front_fov_deg", cfg.front_fov_deg);
    cfg.apf_attractive_gain =
      declare_parameter<double>("apf_attractive_gain", cfg.apf_attractive_gain);
    cfg.apf_repulsive_gain =
      declare_parameter<double>("apf_repulsive_gain", cfg.apf_repulsive_gain);
    cfg.apf_influence_distance =
      declare_parameter<double>("apf_influence_distance", cfg.apf_influence_distance);
    cfg.gap_threshold_distance =
      declare_parameter<double>("gap_threshold_distance", cfg.gap_threshold_distance);
    cfg.stuck_timeout_steps =
      static_cast<int>(declare_parameter<int64_t>("stuck_timeout_steps", cfg.stuck_timeout_steps));
    cfg.trap_timeout_steps =
      static_cast<int>(declare_parameter<int64_t>("trap_timeout_steps", cfg.trap_timeout_steps));
    cfg.robot_radius = declare_parameter<double>("robot_radius", cfg.robot_radius);
    cfg.goal_tolerance = declare_parameter<double>("goal_tolerance", cfg.goal_tolerance);

    if (const std::string error = validate(cfg); !error.empty()) {
      throw std::invalid_argument("invalid parameters: " + error);
    }
    planner_ = std::make_unique<AvoidancePlanner>(cfg);
    for (const auto & wp : parse_waypoints(waypoints)) {
      goals_.push_back(wp);
    }

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {on_scan(*msg);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {on_odom(*msg);});
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {on_goal(*msg);});
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/navigation/markers", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/navigation/status", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/navigation/path", 10);
    pause_srv_ = create_service<std_srvs::srv::SetBool>(
      "/navigation/set_paused",
      [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
        std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        paused_ = req->data;
        if (paused_) {
          publish_stop();
        }
        res->success = true;
        res->message = paused_ ? "Navigation paused." : "Navigation resumed.";
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
      });
    clear_srv_ = create_service<std_srvs::srv::Trigger>(
      "/navigation/clear_goals",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        goals_.clear();
        holding_ = false;
        planner_->reset_state();
        res->success = true;
        res->message = "Goals cleared; robot returns to free exploration.";
      });
    watchdog_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {on_watchdog();});
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return on_parameters(params);});

    RCLCPP_INFO(
      get_logger(),
      "Obstacle avoidance node (C++) started [algorithm=%s, safe=%.2f m, critical=%.2f m, "
      "waypoints=%zu]",
      cfg.algorithm.c_str(), cfg.safe_distance, cfg.critical_distance, goals_.size());
  }

  // Returns an error message for an invalid configuration, or "" if valid.
  static std::string validate(const PlannerConfig & c)
  {
    if (std::find(kAlgorithms.begin(), kAlgorithms.end(), c.algorithm) == kAlgorithms.end()) {
      return "algorithm must be one of sector, apf, follow_the_gap";
    }
    const std::vector<std::pair<const char *, double>> positive{
      {"safe_distance", c.safe_distance}, {"critical_distance", c.critical_distance},
      {"max_linear_speed", c.max_linear_speed}, {"max_angular_speed", c.max_angular_speed},
      {"front_fov_deg", c.front_fov_deg}, {"goal_tolerance", c.goal_tolerance}};
    for (const auto & [name, value] : positive) {
      if (value <= 0.0) {
        return std::string(name) + " must be > 0";
      }
    }
    if (c.critical_distance >= c.safe_distance) {
      return "critical_distance must be smaller than safe_distance";
    }
    if (c.min_linear_speed > c.max_linear_speed) {
      return "min_linear_speed must not exceed max_linear_speed";
    }
    if (c.recovery_linear_speed > 0.0) {
      return "recovery_linear_speed must be <= 0 (it reverses the robot)";
    }
    return "";
  }

  void publish_stop()
  {
    if (cmd_pub_) {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
    }
  }

private:
  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    PlannerConfig candidate = planner_->config();
    std::optional<std::vector<Waypoint>> waypoints;
    try {
      for (const auto & p : params) {
        const std::string & n = p.get_name();
        if (n == "algorithm") {
          candidate.algorithm = p.as_string();
        } else if (n == "safe_distance") {
          candidate.safe_distance = p.as_double();
        } else if (n == "critical_distance") {
          candidate.critical_distance = p.as_double();
        } else if (n == "max_linear_speed") {
          candidate.max_linear_speed = p.as_double();
        } else if (n == "min_linear_speed") {
          candidate.min_linear_speed = p.as_double();
        } else if (n == "recovery_linear_speed") {
          candidate.recovery_linear_speed = p.as_double();
        } else if (n == "max_angular_speed") {
          candidate.max_angular_speed = p.as_double();
        } else if (n == "front_fov_deg") {
          candidate.front_fov_deg = p.as_double();
        } else if (n == "apf_attractive_gain") {
          candidate.apf_attractive_gain = p.as_double();
        } else if (n == "apf_repulsive_gain") {
          candidate.apf_repulsive_gain = p.as_double();
        } else if (n == "apf_influence_distance") {
          candidate.apf_influence_distance = p.as_double();
        } else if (n == "gap_threshold_distance") {
          candidate.gap_threshold_distance = p.as_double();
        } else if (n == "stuck_timeout_steps") {
          candidate.stuck_timeout_steps = static_cast<int>(p.as_int());
        } else if (n == "trap_timeout_steps") {
          candidate.trap_timeout_steps = static_cast<int>(p.as_int());
        } else if (n == "robot_radius") {
          candidate.robot_radius = p.as_double();
        } else if (n == "goal_tolerance") {
          candidate.goal_tolerance = p.as_double();
        } else if (n == "waypoints") {
          waypoints = parse_waypoints(p.as_string());
        } else if (n == "publish_markers") {
          publish_markers_ = p.as_bool();
        } else if (n == "loop_waypoints") {
          loop_waypoints_ = p.as_bool();
        } else if (n == "scan_timeout_s") {
          scan_timeout_s_ = p.as_double();
        } else if (n == "odom_frame") {
          odom_frame_ = p.as_string();
        }
      }
    } catch (const std::exception & e) {
      result.successful = false;
      result.reason = e.what();
      return result;
    }
    if (const std::string error = validate(candidate); !error.empty()) {
      result.successful = false;
      result.reason = error;
      return result;
    }
    planner_->config() = candidate;
    if (waypoints) {
      goals_.assign(waypoints->begin(), waypoints->end());
      holding_ = false;
      planner_->reset_state();
    }
    result.successful = true;
    RCLCPP_INFO(
      get_logger(), "Parameters updated: algorithm=%s, safe_distance=%.2f m",
      candidate.algorithm.c_str(), candidate.safe_distance);
    return result;
  }

  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    const double yaw =
      std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    pose_ = std::array<double, 3>{p.x, p.y, yaw};

    if (last_path_xy_ && std::hypot(p.x - last_path_xy_->first, p.y - last_path_xy_->second) <
      0.03)
    {
      return;
    }
    last_path_xy_ = Waypoint{p.x, p.y};
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header = msg.header;
    stamped.pose = msg.pose.pose;
    path_.header = msg.header;
    path_.poses.push_back(stamped);
    if (path_.poses.size() > 3000) {
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() + 500);
    }
    path_pub_->publish(path_);
  }

  void on_goal(const geometry_msgs::msg::PoseStamped & msg)
  {
    const std::string frame = msg.header.frame_id.empty() ? odom_frame_ : msg.header.frame_id;
    if (frame != odom_frame_) {
      RCLCPP_WARN(
        get_logger(), "Ignoring goal in frame \"%s\": set the RViz fixed frame to \"%s\"",
        frame.c_str(), odom_frame_.c_str());
      return;
    }
    goals_.assign(1, Waypoint{msg.pose.position.x, msg.pose.position.y});
    holding_ = false;
    planner_->reset_state();
    RCLCPP_INFO(get_logger(), "New goal: (%.2f, %.2f)", msg.pose.position.x, msg.pose.position.y);
  }

  // Stop the robot if LiDAR data goes stale (sensor or bridge failure).
  void on_watchdog()
  {
    if (!last_scan_time_) {
      return;
    }
    const double age = (now() - *last_scan_time_).seconds();
    if (age > scan_timeout_s_ && !watchdog_tripped_) {
      watchdog_tripped_ = true;
      publish_stop();
      RCLCPP_ERROR(
        get_logger(), "No LaserScan for %.2f s (> %.2f s): stopping robot", age, scan_timeout_s_);
    }
  }

  void on_scan(const sensor_msgs::msg::LaserScan & msg)
  {
    last_scan_time_ = now();
    if (watchdog_tripped_) {
      watchdog_tripped_ = false;
      RCLCPP_INFO(get_logger(), "LaserScan stream restored");
    }

    std::optional<double> goal_distance;
    double goal_angle = 0.0;
    if (!goals_.empty() && pose_) {
      const auto [gx, gy] = goals_.front();
      const auto [x, y, yaw] = *pose_;
      goal_distance = std::hypot(gx - x, gy - y);
      const double bearing = std::atan2(gy - y, gx - x) - yaw;
      goal_angle = std::atan2(std::sin(bearing), std::cos(bearing));
    }

    ranges_.assign(msg.ranges.begin(), msg.ranges.end());
    const ScanInput scan{
      ranges_, msg.angle_min, msg.angle_increment,
      msg.range_min > 0.0F ? msg.range_min : 0.05, msg.range_max > 0.0F ? msg.range_max : 3.5};
    AvoidanceDecision decision =
      planner_->plan(scan, paused_ || holding_, goal_distance, goal_angle);
    if (holding_ && !paused_) {
      decision.state = NavigationState::kGoalReached;
    }

    if (decision.state == NavigationState::kGoalReached && !goals_.empty()) {
      const Waypoint reached = goals_.front();
      goals_.pop_front();
      ++goals_reached_;
      if (loop_waypoints_) {
        goals_.push_back(reached);
      }
      holding_ = goals_.empty();
      RCLCPP_INFO(
        get_logger(), "Goal (%.2f, %.2f) reached [%d total, %zu remaining]", reached.first,
        reached.second, goals_reached_, goals_.size());
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = decision.linear_x;
    twist.angular.z = decision.angular_z;
    cmd_pub_->publish(twist);

    publish_status(decision, goal_distance);
    if (publish_markers_) {
      publish_markers(
        decision, msg.header.frame_id.empty() ? "base_scan" : msg.header.frame_id,
        msg.header.stamp);
    }
    if (!last_logged_state_ || *last_logged_state_ != decision.state) {
      last_logged_state_ = decision.state;
      RCLCPP_INFO(
        get_logger(), "State -> %s | front=%.2f m | v=%.2f m/s, w=%+.2f rad/s",
        to_string(decision.state), decision.min_front_distance, decision.linear_x,
        decision.angular_z);
    }
  }

  void publish_status(const AvoidanceDecision & d, std::optional<double> goal_distance)
  {
    std::ostringstream json;
    json << "{\"state\": \"" << to_string(d.state) << "\", \"algorithm\": \""
         << planner_->config().algorithm << "\", \"linear_x\": " << json_number(d.linear_x, 3)
         << ", \"angular_z\": " << json_number(d.angular_z, 3)
         << ", \"min_front_distance\": " << json_number(d.min_front_distance, 3)
         << ", \"target_heading_deg\": " << json_number(d.target_heading_rad * 180.0 / kPi, 1)
         << ", \"goal\": ";
    if (goals_.empty()) {
      json << "null";
    } else {
      json << "[" << goals_.front().first << ", " << goals_.front().second << "]";
    }
    json << ", \"goal_distance\": "
         << (goal_distance ? json_number(*goal_distance, 3) : std::string("null"))
         << ", \"goals_remaining\": " << goals_.size()
         << ", \"goals_reached\": " << goals_reached_ << ", \"sectors\": {";
    if (d.has_sectors) {
      for (int s = 0; s < kNumSectors; ++s) {
        json << (s ? ", " : "") << "\"" << kSectorNames[s] << "\": "
             << json_number(d.sector_distances[s], 3);
      }
    }
    json << "}}";
    std_msgs::msg::String msg;
    msg.data = json.str();
    status_pub_->publish(msg);
  }

  void publish_markers(
    const AvoidanceDecision & d, const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp)
  {
    const PlannerConfig & cfg = planner_->config();
    visualization_msgs::msg::MarkerArray markers;
    auto base = [&](const char * ns, int id, int type, const std::string & frame) {
        Marker m;
        m.header.frame_id = frame;
        m.header.stamp = stamp;
        m.ns = ns;
        m.id = id;
        m.type = type;
        m.action = Marker::ADD;
        m.pose.orientation.w = 1.0;
        return m;
      };

    Marker arrow = base("navigation_heading", 0, Marker::ARROW, frame_id);
    arrow.scale.x = 0.03;
    arrow.scale.y = 0.06;
    arrow.scale.z = 0.06;
    arrow.points = {point(0.0, 0.0, 0.05), point(
        0.45 * std::cos(d.target_heading_rad), 0.45 * std::sin(d.target_heading_rad), 0.05)};
    arrow.color = state_color(d.state, 0.95);
    markers.markers.push_back(arrow);

    Marker zone = base("safety_zone", 1, Marker::CYLINDER, frame_id);
    zone.scale.x = zone.scale.y = 2.0 * cfg.safe_distance;
    zone.scale.z = 0.01;
    zone.pose.position.z = 0.01;
    zone.color = state_color(d.state, 0.15);
    markers.markers.push_back(zone);

    // Sector wedges: each drawn out to its measured clearance and coloured by it.
    Marker wedges = base("sectors", 2, Marker::TRIANGLE_LIST, frame_id);
    wedges.scale.x = wedges.scale.y = wedges.scale.z = 1.0;
    wedges.color = rgba(1.0, 1.0, 1.0, 1.0);  // RViz multiplies vertex alpha by this
    if (d.has_sectors) {
      constexpr int kSteps = 6;
      for (const auto & [sector, bounds] : kWedges) {
        const double dist = std::min(d.sector_distances[sector], 3.5);
        const ColorRGBA color = clearance_color(dist, cfg.critical_distance, cfg.safe_distance);
        for (int k = 0; k < kSteps; ++k) {
          const double a0 = (bounds.first + (bounds.second - bounds.first) * k / kSteps) * kPi /
            180.0;
          const double a1 =
            (bounds.first + (bounds.second - bounds.first) * (k + 1) / kSteps) * kPi / 180.0;
          wedges.points.push_back(point(0.0, 0.0, 0.02));
          wedges.points.push_back(point(dist * std::cos(a0), dist * std::sin(a0), 0.02));
          wedges.points.push_back(point(dist * std::cos(a1), dist * std::sin(a1), 0.02));
          wedges.colors.insert(wedges.colors.end(), 3, color);
        }
      }
    }
    markers.markers.push_back(wedges);

    Marker label = base("state_label", 3, Marker::TEXT_VIEW_FACING, frame_id);
    label.pose.position.z = 0.45;
    label.scale.z = 0.12;
    label.color = state_color(d.state, 1.0);
    label.text = std::string(to_string(d.state)) + " [" + cfg.algorithm + ", C++]";
    markers.markers.push_back(label);

    Marker goals = base("goals", 4, Marker::SPHERE_LIST, odom_frame_);
    goals.scale.x = goals.scale.y = goals.scale.z = 0.18;
    goals.color = rgba(1.0, 1.0, 1.0, 1.0);
    for (std::size_t i = 0; i < goals_.size(); ++i) {
      goals.points.push_back(point(goals_[i].first, goals_[i].second, 0.09));
      goals.colors.push_back(i == 0 ? rgba(1.0, 0.8, 0.1, 1.0) : rgba(0.6, 0.6, 0.6, 0.7));
    }
    if (goals_.empty()) {
      goals.action = Marker::DELETE;
    }
    markers.markers.push_back(goals);

    marker_pub_->publish(markers);
  }


  std::unique_ptr<AvoidancePlanner> planner_;
  std::vector<double> ranges_;  // reused buffer for scan ranges as doubles
  std::deque<Waypoint> goals_;
  std::optional<std::array<double, 3>> pose_;
  std::optional<Waypoint> last_path_xy_;
  std::optional<rclcpp::Time> last_scan_time_;
  std::optional<NavigationState> last_logged_state_;
  nav_msgs::msg::Path path_;
  std::string odom_frame_;
  double scan_timeout_s_ = 0.5;
  bool publish_markers_ = true;
  bool loop_waypoints_ = false;
  bool paused_ = false;
  bool holding_ = false;  // true after the last goal: stop until a new goal arrives
  bool watchdog_tripped_ = false;
  int goals_reached_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace my_robot_navigation_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<my_robot_navigation_cpp::ObstacleAvoidanceNode>();
  // Send a zero velocity on Ctrl-C while the context is still valid, so the robot
  // doesn't keep driving with the last command.
  rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
    [node]() {node->publish_stop();});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
