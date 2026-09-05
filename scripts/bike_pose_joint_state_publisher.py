#!/usr/bin/env python3
"""Publish the fixed position-joint state of a transformed physical robot."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from bike_transform_sequence import BIKE_FINAL, JOINT_NAMES


class BikePoseJointStatePublisher(Node):
    def __init__(self):
        super().__init__('bike_pose_joint_state_publisher')
        self.declare_parameter('publish_rate_hz', 10.0)
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        if publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero')

        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_pose)
        self.get_logger().info(
            f'Publishing the fixed bike pose for {len(JOINT_NAMES)} position joints '
            f'at {publish_rate_hz:.1f} Hz'
        )

    def publish_pose(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = list(BIKE_FINAL)
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = BikePoseJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
