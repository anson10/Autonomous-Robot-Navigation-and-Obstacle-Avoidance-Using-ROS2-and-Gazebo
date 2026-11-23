#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Anson Antony

"""ROS2 obstacle avoidance node: LiDAR planner, goal navigation, RViz markers, telemetry."""

from collections import deque
import dataclasses
import json
import math
from typing import Deque, List, Optional, Tuple

from geometry_msgs.msg import Point, PoseStamped, Twist
from my_robot_navigation.avoidance_planner import (
    AvoidanceDecision,
    AvoidancePlanner,
    NavigationState,
    PlannerConfig,
)
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray

ALGORITHMS = ('sector', 'apf', 'follow_the_gap')

# Parameters that map one-to-one onto PlannerConfig fields.
PLANNER_PARAMS = {
    'algorithm': 'sector',
    'safe_distance': 0.65,
    'critical_distance': 0.30,
    'max_linear_speed': 0.22,
    'min_linear_speed': 0.05,
    'recovery_linear_speed': -0.10,
    'max_angular_speed': 1.2,
    'front_fov_deg': 120.0,
    'apf_attractive_gain': 1.0,
    'apf_repulsive_gain': 0.25,
    'apf_influence_distance': 1.2,
    'gap_threshold_distance': 0.80,
    'stuck_timeout_steps': 25,
    'trap_timeout_steps': 35,
    'robot_radius': 0.14,
    'goal_tolerance': 0.22,
}

# Sector wedges drawn in RViz: name -> (start_deg, end_deg).
SECTOR_WEDGES = {
    'right': (-90.0, -50.0),
    'front_right': (-50.0, -15.0),
    'front': (-15.0, 15.0),
    'front_left': (15.0, 50.0),
    'left': (50.0, 90.0),
}


def parse_waypoints(text: str) -> List[Tuple[float, float]]:
    """Parse 'x1,y1; x2,y2; ...' into a list of (x, y) tuples."""
    points: List[Tuple[float, float]] = []
    for chunk in text.replace('\n', ';').split(';'):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = [p.strip() for p in chunk.split(',')]
        if len(parts) != 2:
            raise ValueError(f'waypoint "{chunk}" must be "x,y"')
        points.append((float(parts[0]), float(parts[1])))
    return points


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw (rotation about z) from a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def clearance_color(distance: float, critical: float, safe: float) -> ColorRGBA:
    """Red at/below critical distance, through yellow, to green at 2x safe distance."""
    t = (distance - critical) / max(2.0 * safe - critical, 1e-3)
    t = max(0.0, min(1.0, t))
    return ColorRGBA(r=min(1.0, 2.0 * (1.0 - t)), g=min(1.0, 2.0 * t), b=0.15, a=0.35)


class ObstacleAvoidanceNode(Node):
    """Reactive obstacle avoidance with optional goal / waypoint navigation."""

    def __init__(self) -> None:
        """Declare parameters, then create publishers, subscribers and services."""
        super().__init__('obstacle_avoidance_node')

        read_only = ParameterDescriptor(read_only=True)
        self.declare_parameter('scan_topic', '/scan', read_only)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel', read_only)
        self.declare_parameter('odom_topic', '/odom', read_only)
        self.declare_parameter('goal_topic', '/goal_pose', read_only)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('scan_timeout_s', 0.5)
        self.declare_parameter('waypoints', '')
        self.declare_parameter('loop_waypoints', False)
        for name, default in PLANNER_PARAMS.items():
            self.declare_parameter(name, default)

        error = self._validate({
            name: self.get_parameter(name).value for name in PLANNER_PARAMS
        })
        if error:
            raise ValueError(f'invalid parameters: {error}')
        self.planner = AvoidancePlanner(PlannerConfig(**{
            name: self.get_parameter(name).value for name in PLANNER_PARAMS
        }))

        self.paused = False
        self.pose: Optional[Tuple[float, float, float]] = None
        self.goals: Deque[Tuple[float, float]] = deque()
        self.goals_reached = 0
        self.holding = False  # True after the last goal: stop until a new goal
        self.path = Path()
        self._last_path_xy: Optional[Tuple[float, float]] = None
        self._last_scan_time = None
        self._watchdog_tripped = False
        self._last_logged_state: Optional[NavigationState] = None
        self._load_waypoints(str(self.get_parameter('waypoints').value))

        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self.lidar_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            self._goal_callback,
            10,
        )
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/navigation/markers', 10)
        self.status_pub = self.create_publisher(String, '/navigation/status', 10)
        self.path_pub = self.create_publisher(Path, '/navigation/path', 10)
        self.create_service(SetBool, '/navigation/set_paused', self._handle_set_paused)
        self.create_service(Trigger, '/navigation/clear_goals', self._handle_clear_goals)
        self.create_timer(0.1, self._watchdog)
        self.add_on_set_parameters_callback(self._on_parameters_changed)

        cfg = self.planner.config
        self.get_logger().info(
            f'Obstacle avoidance node started [algorithm={cfg.algorithm}, '
            f'safe={cfg.safe_distance:.2f} m, critical={cfg.critical_distance:.2f} m, '
            f'waypoints={len(self.goals)}]'
        )

    @staticmethod
    def _validate(values: dict) -> str:
        """Return an error message for an invalid parameter set, or '' if valid."""
        if values.get('algorithm') not in ALGORITHMS:
            return f"algorithm must be one of {', '.join(ALGORITHMS)}"
        for name in ('safe_distance', 'critical_distance', 'max_linear_speed',
                     'max_angular_speed', 'front_fov_deg', 'goal_tolerance'):
            if float(values[name]) <= 0.0:
                return f'{name} must be > 0'
        if float(values['critical_distance']) >= float(values['safe_distance']):
            return 'critical_distance must be smaller than safe_distance'
        if float(values['min_linear_speed']) > float(values['max_linear_speed']):
            return 'min_linear_speed must not exceed max_linear_speed'
        if float(values['recovery_linear_speed']) > 0.0:
            return 'recovery_linear_speed must be <= 0 (it reverses the robot)'
        return ''

    def _on_parameters_changed(self, params: List[Parameter]) -> SetParametersResult:
        """Validate and apply runtime parameter changes atomically."""
        cfg = self.planner.config
        candidate = {name: getattr(cfg, name) for name in PLANNER_PARAMS}
        waypoints = None
        for p in params:
            if p.name in candidate:
                candidate[p.name] = p.value
            elif p.name == 'waypoints':
                try:
                    waypoints = parse_waypoints(str(p.value))
                except ValueError as exc:
                    return SetParametersResult(successful=False, reason=str(exc))
        error = self._validate(candidate)
        if error:
            return SetParametersResult(successful=False, reason=error)
        for name, value in candidate.items():
            setattr(cfg, name, value)
        if waypoints is not None:
            self.goals = deque(waypoints)
            self.holding = False
            self.planner.reset_state()
        self.get_logger().info(
            f'Parameters updated: algorithm={cfg.algorithm}, '
            f'safe_distance={cfg.safe_distance:.2f} m'
        )
        return SetParametersResult(successful=True)

    def _load_waypoints(self, text: str) -> None:
        try:
            self.goals = deque(parse_waypoints(text))
        except ValueError as exc:
            raise ValueError(f'invalid waypoints parameter: {exc}') from exc

    def _odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        if self._last_path_xy is None or math.hypot(
            p.x - self._last_path_xy[0], p.y - self._last_path_xy[1]
        ) >= 0.03:
            self._last_path_xy = (p.x, p.y)
            stamped = PoseStamped()
            stamped.header = msg.header
            stamped.pose = msg.pose.pose
            self.path.header = msg.header
            self.path.poses.append(stamped)
            if len(self.path.poses) > 3000:
                del self.path.poses[:500]
            self.path_pub.publish(self.path)

    def _goal_callback(self, msg: PoseStamped) -> None:
        frame = msg.header.frame_id or self.get_parameter('odom_frame').value
        if frame != self.get_parameter('odom_frame').value:
            self.get_logger().warn(
                f'Ignoring goal in frame "{frame}": set the RViz fixed frame to '
                f'"{self.get_parameter("odom_frame").value}"'
            )
            return
        self.goals = deque([(msg.pose.position.x, msg.pose.position.y)])
        self.holding = False
        self.planner.reset_state()
        self.get_logger().info(
            f'New goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )

    def _handle_set_paused(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self.paused = bool(request.data)
        if self.paused:
            self.publish_stop()
        response.success = True
        response.message = 'Navigation paused.' if self.paused else 'Navigation resumed.'
        self.get_logger().info(response.message)
        return response

    def _handle_clear_goals(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self.goals.clear()
        self.holding = False
        self.planner.reset_state()
        response.success = True
        response.message = 'Goals cleared; robot returns to free exploration.'
        return response

    def _watchdog(self) -> None:
        """Stop the robot if LiDAR data goes stale (sensor or bridge failure)."""
        if self._last_scan_time is None:
            return
        age = (self.get_clock().now() - self._last_scan_time).nanoseconds * 1e-9
        timeout = float(self.get_parameter('scan_timeout_s').value)
        if age > timeout and not self._watchdog_tripped:
            self._watchdog_tripped = True
            self.publish_stop()
            self.get_logger().error(
                f'No LaserScan for {age:.2f} s (> {timeout:.2f} s): stopping robot'
            )

    def _goal_relative(self) -> Tuple[Optional[float], float]:
        """Distance and bearing (robot frame) to the active goal, if any."""
        if not self.goals or self.pose is None:
            return None, 0.0
        gx, gy = self.goals[0]
        x, y, yaw = self.pose
        bearing = math.atan2(gy - y, gx - x) - yaw
        return math.hypot(gx - x, gy - y), math.atan2(math.sin(bearing), math.cos(bearing))

    def lidar_callback(self, msg: LaserScan) -> None:
        """Run one planning cycle per scan and publish command, status and markers."""
        self._last_scan_time = self.get_clock().now()
        if self._watchdog_tripped:
            self._watchdog_tripped = False
            self.get_logger().info('LaserScan stream restored')

        goal_distance, goal_angle = self._goal_relative()
        decision = self.planner.plan(
            ranges=msg.ranges,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_min=msg.range_min if msg.range_min > 0.0 else 0.05,
            range_max=msg.range_max if msg.range_max > 0.0 else 3.5,
            paused=self.paused or self.holding,
            goal_distance=goal_distance,
            goal_angle_rad=goal_angle,
        )
        if self.holding and not self.paused:
            decision = dataclasses.replace(decision, state=NavigationState.GOAL_REACHED)

        if decision.state == NavigationState.GOAL_REACHED and self.goals:
            reached = self.goals.popleft()
            self.goals_reached += 1
            if bool(self.get_parameter('loop_waypoints').value):
                self.goals.append(reached)
            self.holding = not self.goals
            self.get_logger().info(
                f'Goal ({reached[0]:.2f}, {reached[1]:.2f}) reached '
                f'[{self.goals_reached} total, {len(self.goals)} remaining]'
            )

        twist = Twist()
        twist.linear.x = float(decision.linear_x)
        twist.angular.z = float(decision.angular_z)
        self.publisher.publish(twist)

        self._publish_status(decision, goal_distance)
        if bool(self.get_parameter('publish_markers').value):
            self._publish_markers(decision, msg.header.frame_id or 'base_scan', msg.header.stamp)

        if decision.state != self._last_logged_state:
            self._last_logged_state = decision.state
            self.get_logger().info(
                f'State -> {decision.state.value} | front={decision.min_front_distance:.2f} m '
                f'| v={decision.linear_x:.2f} m/s, w={decision.angular_z:+.2f} rad/s'
            )

    def _publish_status(self, decision: AvoidanceDecision, goal_distance) -> None:
        payload = {
            'state': decision.state.value,
            'algorithm': self.planner.config.algorithm,
            'linear_x': round(decision.linear_x, 3),
            'angular_z': round(decision.angular_z, 3),
            'min_front_distance': round(decision.min_front_distance, 3),
            'target_heading_deg': round(math.degrees(decision.target_heading_rad), 1),
            'goal': list(self.goals[0]) if self.goals else None,
            'goal_distance': None if goal_distance is None else round(goal_distance, 3),
            'goals_remaining': len(self.goals),
            'goals_reached': self.goals_reached,
            'sectors': {k: round(v, 3) for k, v in decision.sector_distances.items()},
        }
        self.status_pub.publish(String(data=json.dumps(payload)))

    def _publish_markers(self, decision: AvoidanceDecision, frame_id: str, stamp) -> None:
        cfg = self.planner.config
        r, g, b = self._state_color(decision.state)
        markers = MarkerArray()

        def base(ns: str, mid: int, mtype: int, frame: str = frame_id) -> Marker:
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = stamp
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            return m

        arrow = base('navigation_heading', 0, Marker.ARROW)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.03, 0.06, 0.06
        arrow.points = [
            Point(x=0.0, y=0.0, z=0.05),
            Point(
                x=0.45 * math.cos(decision.target_heading_rad),
                y=0.45 * math.sin(decision.target_heading_rad),
                z=0.05,
            ),
        ]
        arrow.color = ColorRGBA(r=r, g=g, b=b, a=0.95)
        markers.markers.append(arrow)

        zone = base('safety_zone', 1, Marker.CYLINDER)
        zone.scale.x = zone.scale.y = 2.0 * cfg.safe_distance
        zone.scale.z = 0.01
        zone.pose.position.z = 0.01
        zone.color = ColorRGBA(r=r, g=g, b=b, a=0.15)
        markers.markers.append(zone)

        # Sector wedges: each drawn out to its measured clearance and colored by it.
        wedges = base('sectors', 2, Marker.TRIANGLE_LIST)
        wedges.scale.x = wedges.scale.y = wedges.scale.z = 1.0
        # RViz multiplies per-vertex alpha by marker.color.a, so this must be opaque.
        wedges.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        for name, (lo, hi) in SECTOR_WEDGES.items():
            dist = min(decision.sector_distances.get(name, 3.5), 3.5)
            color = clearance_color(dist, cfg.critical_distance, cfg.safe_distance)
            steps = 6
            for k in range(steps):
                a0 = math.radians(lo + (hi - lo) * k / steps)
                a1 = math.radians(lo + (hi - lo) * (k + 1) / steps)
                wedges.points += [
                    Point(x=0.0, y=0.0, z=0.02),
                    Point(x=dist * math.cos(a0), y=dist * math.sin(a0), z=0.02),
                    Point(x=dist * math.cos(a1), y=dist * math.sin(a1), z=0.02),
                ]
                wedges.colors += [color, color, color]
        markers.markers.append(wedges)

        label = base('state_label', 3, Marker.TEXT_VIEW_FACING)
        label.pose.position.z = 0.45
        label.scale.z = 0.12
        label.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
        label.text = f'{decision.state.value} [{cfg.algorithm}]'
        markers.markers.append(label)

        odom_frame = str(self.get_parameter('odom_frame').value)
        goal_marker = base('goals', 4, Marker.SPHERE_LIST, odom_frame)
        goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.18
        goal_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        for i, (gx, gy) in enumerate(self.goals):
            goal_marker.points.append(Point(x=gx, y=gy, z=0.09))
            goal_marker.colors.append(
                ColorRGBA(r=1.0, g=0.8, b=0.1, a=1.0) if i == 0
                else ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.7)
            )
        if not self.goals:
            goal_marker.action = Marker.DELETE
        markers.markers.append(goal_marker)

        self.marker_pub.publish(markers)

    @staticmethod
    def _state_color(state: NavigationState) -> Tuple[float, float, float]:
        return {
            NavigationState.CLEAR: (0.18, 0.80, 0.44),
            NavigationState.AVOIDING: (0.95, 0.61, 0.07),
            NavigationState.WALL_FOLLOW: (0.00, 0.74, 0.83),
            NavigationState.CRITICAL_STOP: (0.91, 0.30, 0.24),
            NavigationState.RECOVERY: (0.61, 0.35, 0.71),
            NavigationState.GOAL_REACHED: (0.20, 0.60, 0.86),
        }.get(state, (0.58, 0.65, 0.65))

    def publish_stop(self) -> None:
        """Publish a zero-velocity command."""
        self.publisher.publish(Twist())


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for the obstacle avoidance node."""
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
