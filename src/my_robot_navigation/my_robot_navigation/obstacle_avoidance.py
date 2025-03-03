#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        # Subscriber to LIDAR data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # TurtleBot3 LIDAR topic
            self.lidar_callback,
            10
        )
        # Publisher for velocity commands
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',  # TurtleBot3 velocity topic
            10
        )
        self.get_logger().info("Obstacle Avoidance Node Started")

    def lidar_callback(self, msg):
        # Process LIDAR data (360 degrees, 0 = front, 180 = rear)
        ranges = msg.ranges
        min_distance = min(ranges[0:60] + ranges[300:360])  # Check front 120 degrees
        twist = Twist()

        if min_distance < 0.5:  # If obstacle closer than 0.5m
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Turn right
            self.get_logger().info(f"Obstacle detected {min_distance:.2f}m ahead, turning!")
        else:
            twist.linear.x = 0.2  # Move forward
            twist.angular.z = 0.0
            self.get_logger().info("Path clear, moving forward")

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
