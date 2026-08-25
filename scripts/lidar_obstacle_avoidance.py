#!/usr/bin/env python3
"""Low-speed reactive obstacle avoidance for simulation validation."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_avoidance')
        self.declare_parameter('forward_speed', 0.08)
        self.declare_parameter('turn_speed', 0.55)
        self.declare_parameter('stop_distance', 0.65)
        self.declare_parameter('clear_distance', 0.85)
        self.declare_parameter('front_half_angle', 0.35)
        self.declare_parameter('robot_half_width', 0.095)
        self.declare_parameter('safety_margin', 0.04)
        self.declare_parameter('scan_timeout', 0.5)

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.clear_distance = float(self.get_parameter('clear_distance').value)
        self.front_half_angle = float(self.get_parameter('front_half_angle').value)
        self.robot_half_width = float(self.get_parameter('robot_half_width').value)
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)

        if self.forward_speed <= 0.0 or self.turn_speed <= 0.0:
            raise ValueError('forward_speed and turn_speed must be greater than zero')
        if self.stop_distance <= 0.0 or self.clear_distance <= self.stop_distance:
            raise ValueError('clear_distance must be greater than stop_distance')
        if self.front_half_angle <= 0.0 or self.scan_timeout <= 0.0:
            raise ValueError('front_half_angle and scan_timeout must be greater than zero')
        if self.robot_half_width <= 0.0 or self.safety_margin < 0.0:
            raise ValueError('robot_half_width must be positive and safety_margin non-negative')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.1, self.stop_on_scan_timeout)
        self.last_scan_time = None
        self.stopped_for_timeout = True
        self.avoiding = False
        self.turn_direction = 1.0

    @staticmethod
    def sector_values(msg, angle_min, angle_max):
        values = []
        for index, distance in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            if angle_min <= angle <= angle_max:
                values.append(distance if math.isfinite(distance) else msg.range_max)
        return values

    def corridor_distance(self, msg):
        half_width = self.robot_half_width + self.safety_margin
        forward_distances = []
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            angle = msg.angle_min + index * msg.angle_increment
            forward = distance * math.cos(angle)
            lateral = distance * math.sin(angle)
            if forward > 0.0 and abs(lateral) <= half_width:
                forward_distances.append(forward)
        return min(forward_distances) if forward_distances else msg.range_max

    def scan_callback(self, msg):
        front_min = self.corridor_distance(msg)
        if not self.avoiding and front_min <= self.stop_distance:
            left = self.sector_values(msg, self.front_half_angle, 1.40)
            right = self.sector_values(msg, -1.40, -self.front_half_angle)
            left_clearance = sum(left) / len(left) if left else 0.0
            right_clearance = sum(right) / len(right) if right else 0.0
            self.turn_direction = 1.0 if left_clearance >= right_clearance else -1.0
            self.avoiding = True
            direction = 'left' if self.turn_direction > 0.0 else 'right'
            self.get_logger().info(
                f'obstacle at {front_min:.2f} m: turning {direction}'
            )
        elif self.avoiding and front_min >= self.clear_distance:
            self.avoiding = False
            self.get_logger().info(f'path clear at {front_min:.2f} m: moving forward')

        command = Twist()
        if self.avoiding:
            command.angular.z = self.turn_direction * self.turn_speed
        else:
            command.linear.x = self.forward_speed
        self.publisher.publish(command)
        self.last_scan_time = self.get_clock().now()
        self.stopped_for_timeout = False

    def publish_stop(self):
        self.publisher.publish(Twist())

    def stop_on_scan_timeout(self):
        if self.last_scan_time is None or self.stopped_for_timeout:
            return
        elapsed = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if elapsed >= self.scan_timeout:
            self.publish_stop()
            self.stopped_for_timeout = True
            self.get_logger().warn('scan timeout: robot stopped')


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
