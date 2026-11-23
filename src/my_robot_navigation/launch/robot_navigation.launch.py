# SPDX-License-Identifier: MIT
# Copyright (c) 2025-2026 Anson Antony

"""Launch Gazebo (custom arena or stock TurtleBot3 world), the avoidance node and RViz2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Default spawn pose per world (x, y). The arena pose sits left of the pillar ring.
WORLD_SPAWN = {
    'arena': ('-2.2', '0.5'),
    'turtlebot3_world': ('-2.0', '-0.5'),
}

# Launch arguments that override navigation_params.yaml only when given explicitly.
OVERRIDABLE = ('algorithm', 'safe_distance', 'waypoints', 'loop_waypoints')


def _typed(value: str):
    """Convert a launch-argument string to bool / float / str for ROS parameters."""
    lowered = value.lower()
    if lowered in ('true', 'false'):
        return lowered == 'true'
    try:
        return float(value)
    except ValueError:
        return value


def _setup(context, *_args, **_kwargs):
    pkg_share = get_package_share_directory('my_robot_navigation')

    def arg(name: str) -> str:
        return context.launch_configurations.get(name, '')

    use_sim_time = arg('use_sim_time').lower() == 'true'
    actions = []

    if arg('use_gazebo').lower() == 'true':
        gazebo_ros = get_package_share_directory('gazebo_ros')
        tb3_share = get_package_share_directory('turtlebot3_gazebo')
        tb3_launch = os.path.join(tb3_share, 'launch')
        world = arg('world')
        if world == 'arena':
            world_path = os.path.join(pkg_share, 'worlds', 'obstacle_arena.world')
        elif world == 'turtlebot3_world':
            world_path = os.path.join(tb3_share, 'worlds', 'turtlebot3_world.world')
        else:
            world_path = world  # absolute path to any .world file
        default_x, default_y = WORLD_SPAWN.get(world, ('0.0', '0.0'))
        x_pose = arg('x_pose') or default_x
        y_pose = arg('y_pose') or default_y

        actions += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')
                ),
                launch_arguments={'world': world_path}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tb3_launch, 'robot_state_publisher.launch.py')
                ),
                launch_arguments={'use_sim_time': str(use_sim_time).lower()}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tb3_launch, 'spawn_turtlebot3.launch.py')
                ),
                launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
            ),
        ]
        if arg('gui').lower() == 'true':
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')
                    )
                )
            )

    overrides = {'use_sim_time': use_sim_time}
    for name in OVERRIDABLE:
        if arg(name) != '':
            overrides[name] = arg(name) if name == 'waypoints' else _typed(arg(name))

    if arg('impl') == 'cpp':
        package, executable = 'my_robot_navigation_cpp', 'obstacle_avoidance_node'
    else:
        package, executable = 'my_robot_navigation', 'obstacle_avoidance'
    actions.append(
        Node(
            package=package,
            executable=executable,
            name='obstacle_avoidance_node',
            output='screen',
            parameters=[arg('params_file'), overrides],
        )
    )

    if arg('use_rviz').lower() == 'true':
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(pkg_share, 'rviz', 'obstacle_avoidance.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    """Declare launch arguments and defer node creation until they are resolved."""
    pkg_share = get_package_share_directory('my_robot_navigation')
    args = [
        DeclareLaunchArgument(
            'world', default_value='arena',
            description='"arena" (custom pillar arena), "turtlebot3_world", '
                        'or an absolute path to a .world file',
        ),
        DeclareLaunchArgument('x_pose', default_value='', description='Spawn x (m)'),
        DeclareLaunchArgument('y_pose', default_value='', description='Spawn y (m)'),
        DeclareLaunchArgument(
            'impl', default_value='cpp',
            description='Navigation node implementation: cpp (rclcpp) or python (rclpy)',
        ),
        DeclareLaunchArgument('use_gazebo', default_value='true'),
        DeclareLaunchArgument(
            'gui', default_value='true', description='Start the Gazebo client window'
        ),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'navigation_params.yaml'),
        ),
        DeclareLaunchArgument(
            'algorithm', default_value='',
            description='Override: sector | apf | follow_the_gap (default: YAML value)',
        ),
        DeclareLaunchArgument(
            'safe_distance', default_value='', description='Override (m)'
        ),
        DeclareLaunchArgument(
            'waypoints', default_value='',
            description='Override: "x1,y1; x2,y2; ..." in the odom frame',
        ),
        DeclareLaunchArgument(
            'loop_waypoints', default_value='', description='Override: true | false'
        ),
        DeclareLaunchArgument(
            'turtlebot3_model', default_value='burger',
            description='burger | waffle | waffle_pi',
        ),
    ]
    return LaunchDescription(args + [
        SetEnvironmentVariable('TURTLEBOT3_MODEL', LaunchConfiguration('turtlebot3_model')),
        OpaqueFunction(function=_setup),
    ])
