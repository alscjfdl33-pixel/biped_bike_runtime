#!/usr/bin/env python3
"""Publish planar odometry and base attitude from Gazebo ground truth."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


def quaternion_to_euler(x, y, z, w):
    sin_roll_cos_pitch = 2.0 * (w * x + y * z)
    cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)

    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    sin_yaw_cos_pitch = 2.0 * (w * z + x * y)
    cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)
    return roll, pitch, yaw


def roll_pitch_quaternion(roll, pitch):
    half_roll = roll / 2.0
    half_pitch = pitch / 2.0
    sin_roll = math.sin(half_roll)
    cos_roll = math.cos(half_roll)
    sin_pitch = math.sin(half_pitch)
    cos_pitch = math.cos(half_pitch)
    return (
        sin_roll * cos_pitch,
        cos_roll * sin_pitch,
        -sin_roll * sin_pitch,
        cos_roll * cos_pitch,
    )


class GroundTruthAttitude(Node):
    def __init__(self):
        super().__init__('ground_truth_attitude')
        self.declare_parameter('parent_frame', 'base_footprint')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')

        self.parent_frame = str(self.get_parameter('parent_frame').value)
        self.child_frame = str(self.get_parameter('child_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            '/gazebo/ground_truth_odom',
            self.odometry_callback,
            qos_profile_sensor_data,
        )

    def odometry_callback(self, msg):
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = quaternion_to_euler(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        half_yaw = yaw / 2.0
        yaw_z = math.sin(half_yaw)
        yaw_w = math.cos(half_yaw)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.parent_frame
        odom.pose.pose.position.x = msg.pose.pose.position.x
        odom.pose.pose.position.y = msg.pose.pose.position.y
        odom.pose.pose.orientation.z = yaw_z
        odom.pose.pose.orientation.w = yaw_w
        odom.twist.twist.linear.x = msg.twist.twist.linear.x
        odom.twist.twist.linear.y = msg.twist.twist.linear.y
        odom.twist.twist.angular.z = msg.twist.twist.angular.z
        odom.pose.covariance[0] = 1.0e-4
        odom.pose.covariance[7] = 1.0e-4
        odom.pose.covariance[35] = 1.0e-4
        odom.twist.covariance[0] = 1.0e-4
        odom.twist.covariance[7] = 1.0e-4
        odom.twist.covariance[35] = 1.0e-4
        self.odom_publisher.publish(odom)

        planar_transform = TransformStamped()
        planar_transform.header = odom.header
        planar_transform.child_frame_id = self.parent_frame
        planar_transform.transform.translation.x = msg.pose.pose.position.x
        planar_transform.transform.translation.y = msg.pose.pose.position.y
        planar_transform.transform.rotation.z = yaw_z
        planar_transform.transform.rotation.w = yaw_w
        self.tf_broadcaster.sendTransform(planar_transform)

        qx, qy, qz, qw = roll_pitch_quaternion(roll, pitch)

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthAttitude()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
