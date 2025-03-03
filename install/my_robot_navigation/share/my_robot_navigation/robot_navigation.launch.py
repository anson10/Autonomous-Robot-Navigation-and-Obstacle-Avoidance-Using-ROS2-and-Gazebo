from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        # Set the TURTLEBOT3_MODEL environment variable
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        # Launch Gazebo with TurtleBot3
        ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py'],
            output='screen'
        ),
        # Launch the obstacle avoidance node
        Node(
            package='my_robot_navigation',
            executable='obstacle_avoidance',
            name='obstacle_avoidance_node',
            output='screen'
        )
    ])
