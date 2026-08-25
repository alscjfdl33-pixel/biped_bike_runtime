#!/usr/bin/env python3
"""Generate a slow, open-loop trajectory for climbing small known stairs.

This is an initial Gazebo-oriented stair gait.  One stair cycle moves the lead
foot onto the next tread, raises the pelvis, and then brings the trailing foot
onto the same tread.  It intentionally uses small defaults; validate every
phase in simulation before using it on hardware.
"""

import dataclasses
import math
from typing import Iterable, List, Sequence, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from op3_walker import (
    JOINT_AXIS_DIR,
    LEG_LENGTH,
    ik_for_left_leg,
    ik_for_right_leg,
)


JOINT_NAMES = [
    "l_hip_yaw_jnt",
    "l_hip_roll_jnt",
    "l_hip_pitch_jnt",
    "l_knee_pitch_jnt",
    "l_ankle_pitch_jnt",
    "l_foot_roll_jnt",
    "r_hip_yaw_jnt",
    "r_hip_roll_jnt",
    "r_hip_pitch_jnt",
    "r_knee_pitch_jnt",
    "r_ankle_pitch_jnt",
    "r_foot_roll_jnt",
    "arm_base_yaw_jnt",
    "arm_shoulder_pitch_jnt",
    "arm_elbow_pitch_jnt",
    "arm_wrist_pitch_jnt",
    "arm_wrist_roll_jnt",
]


@dataclasses.dataclass(frozen=True)
class FootTarget:
    x: float
    y: float
    z: float
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclasses.dataclass(frozen=True)
class StairPose:
    left: FootTarget
    right: FootTarget
    label: str


def smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def interpolate_foot(start: FootTarget, end: FootTarget, alpha: float) -> FootTarget:
    alpha = smoothstep(alpha)
    values = [
        getattr(start, name) + alpha * (getattr(end, name) - getattr(start, name))
        for name in ("x", "y", "z", "roll", "pitch", "yaw")
    ]
    return FootTarget(*values)


def interpolate_pose(start: StairPose, end: StairPose, alpha: float) -> StairPose:
    return StairPose(
        left=interpolate_foot(start.left, end.left, alpha),
        right=interpolate_foot(start.right, end.right, alpha),
        label=end.label,
    )


def make_stair_cycle(
    *,
    lead_leg: str,
    nominal_x: float,
    nominal_z: float,
    foot_y_offset: float,
    step_height: float,
    step_forward: float,
    foot_clearance: float,
    lateral_shift: float,
) -> List[StairPose]:
    """Return Cartesian phase endpoints for ascending one stair."""
    if lead_leg not in {"left", "right"}:
        raise ValueError("lead_leg must be 'left' or 'right'")

    left_y = foot_y_offset
    right_y = -foot_y_offset

    def pose(
        left_x: float,
        left_z: float,
        right_x: float,
        right_z: float,
        body_shift_y: float,
        label: str,
    ) -> StairPose:
        # A body shift appears as the opposite common foot displacement in the
        # pelvis frame.  The sign here follows the existing OP3 walker frame.
        return StairPose(
            left=FootTarget(left_x, left_y + body_shift_y, left_z),
            right=FootTarget(right_x, right_y + body_shift_y, right_z),
            label=label,
        )

    neutral = pose(nominal_x, nominal_z, nominal_x, nominal_z, 0.0, "neutral")
    if lead_leg == "left":
        support_shift = -abs(lateral_shift)
        landing_shift = abs(lateral_shift)
        return [
            neutral,
            pose(nominal_x, nominal_z, nominal_x, nominal_z,
                 support_shift, "shift_to_right_support"),
            pose(nominal_x, nominal_z, nominal_x, nominal_z,
                 support_shift, "hold_right_support"),
            pose(nominal_x,
                 nominal_z + step_height + foot_clearance,
                 nominal_x, nominal_z, support_shift, "lift_left_vertical"),
            pose(nominal_x - step_forward,
                 nominal_z + step_height + foot_clearance,
                 nominal_x, nominal_z, support_shift, "advance_left_high"),
            pose(nominal_x - step_forward, nominal_z + step_height,
                 nominal_x, nominal_z, support_shift, "land_left"),
            pose(nominal_x - step_forward, nominal_z + step_height,
                 nominal_x, nominal_z, support_shift, "settle_left_contact"),
            pose(nominal_x, nominal_z,
                 nominal_x + step_forward, nominal_z - step_height,
                 landing_shift, "raise_pelvis_over_left"),
            pose(nominal_x, nominal_z,
                 nominal_x + step_forward,
                 nominal_z + foot_clearance,
                 landing_shift, "lift_right_vertical"),
            pose(nominal_x, nominal_z,
                 nominal_x, nominal_z + foot_clearance,
                 landing_shift, "advance_right_high"),
            pose(nominal_x, nominal_z, nominal_x, nominal_z,
                 landing_shift, "land_right"),
            pose(nominal_x, nominal_z, nominal_x, nominal_z,
                 landing_shift, "settle_right_contact"),
            neutral,
        ]

    support_shift = abs(lateral_shift)
    landing_shift = -abs(lateral_shift)
    return [
        neutral,
        pose(nominal_x, nominal_z, nominal_x, nominal_z,
             support_shift, "shift_to_left_support"),
        pose(nominal_x, nominal_z, nominal_x, nominal_z,
             support_shift, "hold_left_support"),
        pose(nominal_x, nominal_z,
             nominal_x,
             nominal_z + step_height + foot_clearance,
             support_shift, "lift_right_vertical"),
        pose(nominal_x, nominal_z,
             nominal_x - step_forward,
             nominal_z + step_height + foot_clearance,
             support_shift, "advance_right_high"),
        pose(nominal_x, nominal_z,
             nominal_x - step_forward, nominal_z + step_height,
             support_shift, "land_right"),
        pose(nominal_x, nominal_z,
             nominal_x - step_forward, nominal_z + step_height,
             support_shift, "settle_right_contact"),
        pose(nominal_x + step_forward, nominal_z - step_height,
             nominal_x, nominal_z, landing_shift, "raise_pelvis_over_right"),
        pose(nominal_x + step_forward,
             nominal_z + foot_clearance,
             nominal_x, nominal_z, landing_shift, "lift_left_vertical"),
        pose(nominal_x, nominal_z + foot_clearance,
             nominal_x, nominal_z, landing_shift, "advance_left_high"),
        pose(nominal_x, nominal_z, nominal_x, nominal_z,
             landing_shift, "land_left"),
        pose(nominal_x, nominal_z, nominal_x, nominal_z,
             landing_shift, "settle_left_contact"),
        neutral,
    ]


def pose_to_joint_positions(
    pose: StairPose,
    hip_pitch_offset: float,
    arm_shoulder_pitch: float,
) -> List[float]:
    left = ik_for_left_leg(
        pose.left.x,
        pose.left.y,
        pose.left.z,
        pose.left.roll,
        pose.left.pitch,
        pose.left.yaw,
    )
    right = ik_for_right_leg(
        pose.right.x,
        pose.right.y,
        pose.right.z,
        pose.right.roll,
        pose.right.pitch,
        pose.right.yaw,
    )
    if left is None or right is None:
        raise ValueError(f"IK failed at stair phase: {pose.label}")

    # Match the standing forward-lean convention used by OP3WalkingEngine.
    right[2] -= JOINT_AXIS_DIR[2] * hip_pitch_offset
    left[2] -= JOINT_AXIS_DIR[8] * hip_pitch_offset

    positions = np.zeros(17)
    positions[0:6] = left
    positions[6:12] = right
    positions[13] = arm_shoulder_pitch
    return positions.tolist()


def sample_cycle(
    phases: Sequence[StairPose],
    phase_duration: float,
    sample_period: float,
) -> Iterable[Tuple[float, StairPose]]:
    elapsed = 0.0
    for start, end in zip(phases, phases[1:]):
        sample_count = max(2, int(math.ceil(phase_duration / sample_period)))
        for index in range(1, sample_count + 1):
            alpha = index / sample_count
            yield elapsed + alpha * phase_duration, interpolate_pose(start, end, alpha)
        elapsed += phase_duration


class StairClimberNode(Node):
    def __init__(self):
        super().__init__("stair_climber")

        self.declare_parameter("num_stairs", 1)
        self.declare_parameter("first_lead_leg", "left")
        self.declare_parameter("step_height", 0.01)
        # The positive magnitude is converted into the leg IK frame below.  In
        # Gazebo this produces +X chassis travel, matching stairs.sdf.
        self.declare_parameter("step_forward", 0.06)
        self.declare_parameter("foot_clearance", 0.040)
        self.declare_parameter("lateral_shift", 0.035)
        self.declare_parameter("init_x_offset", -0.020)
        self.declare_parameter("init_z_offset", 0.025)
        self.declare_parameter("foot_y_offset", 0.0025)
        self.declare_parameter("hip_pitch_offset_deg", 12.0)
        self.declare_parameter("arm_shoulder_pitch_deg", 20.0)
        self.declare_parameter("startup_duration_sec", 3.0)
        self.declare_parameter("phase_duration_sec", 1.5)
        self.declare_parameter("sample_period_sec", 0.02)
        self.declare_parameter("max_abs_joint_rad", 2.2)

        self.num_stairs = max(1, int(self.get_parameter("num_stairs").value))
        self.first_lead_leg = str(self.get_parameter("first_lead_leg").value).lower()
        self.step_height = float(self.get_parameter("step_height").value)
        self.step_forward = float(self.get_parameter("step_forward").value)
        self.foot_clearance = float(self.get_parameter("foot_clearance").value)
        self.lateral_shift = float(self.get_parameter("lateral_shift").value)
        self.nominal_x = float(self.get_parameter("init_x_offset").value)
        init_z_offset = float(self.get_parameter("init_z_offset").value)
        self.nominal_z = init_z_offset - LEG_LENGTH
        self.foot_y_offset = float(self.get_parameter("foot_y_offset").value)
        self.hip_pitch_offset = math.radians(
            float(self.get_parameter("hip_pitch_offset_deg").value)
        )
        self.arm_shoulder_pitch = math.radians(
            float(self.get_parameter("arm_shoulder_pitch_deg").value)
        )
        self.startup_duration = max(
            0.1, float(self.get_parameter("startup_duration_sec").value)
        )
        self.phase_duration = max(
            0.1, float(self.get_parameter("phase_duration_sec").value)
        )
        self.sample_period = max(
            0.008, float(self.get_parameter("sample_period_sec").value)
        )
        self.max_abs_joint_rad = max(
            0.1, float(self.get_parameter("max_abs_joint_rad").value)
        )

        self._validate_parameters()
        self.publisher = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )
        self.timer = self.create_timer(1.0, self.publish_trajectory)

    def _validate_parameters(self):
        if self.first_lead_leg not in {"left", "right"}:
            raise ValueError("first_lead_leg must be 'left' or 'right'")
        if not 0.0 < self.step_height <= 0.03:
            raise ValueError("step_height must be in (0.0, 0.03] meters")
        if not 0.0 < self.step_forward <= 0.06:
            raise ValueError("step_forward must be in (0.0, 0.06] meters")
        if not 0.005 <= self.foot_clearance <= 0.04:
            raise ValueError("foot_clearance must be in [0.005, 0.04] meters")
        if not 0.0 <= self.lateral_shift <= 0.05:
            raise ValueError("lateral_shift must be in [0.0, 0.05] meters")

    @staticmethod
    def _duration(seconds: float) -> Duration:
        sec = int(seconds)
        nanosec = int(round((seconds - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        return Duration(sec=sec, nanosec=nanosec)

    def _lead_leg_for_stair(self, stair_index: int) -> str:
        if stair_index % 2 == 0:
            return self.first_lead_leg
        return "right" if self.first_lead_leg == "left" else "left"

    def _make_cycle(self, lead_leg: str) -> List[StairPose]:
        return make_stair_cycle(
            lead_leg=lead_leg,
            nominal_x=self.nominal_x,
            nominal_z=self.nominal_z,
            foot_y_offset=self.foot_y_offset,
            step_height=self.step_height,
            step_forward=self.step_forward,
            foot_clearance=self.foot_clearance,
            lateral_shift=self.lateral_shift,
        )

    def publish_trajectory(self):
        self.timer.cancel()
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES

        initial_pose = self._make_cycle(self.first_lead_leg)[0]
        initial_positions = pose_to_joint_positions(
            initial_pose, self.hip_pitch_offset, self.arm_shoulder_pitch
        )
        initial_point = JointTrajectoryPoint()
        initial_point.positions = initial_positions
        initial_point.time_from_start = self._duration(self.startup_duration)
        msg.points.append(initial_point)

        elapsed = self.startup_duration
        largest_angle = max(abs(value) for value in initial_positions)
        phase_labels = []

        for stair_index in range(self.num_stairs):
            lead_leg = self._lead_leg_for_stair(stair_index)
            phases = self._make_cycle(lead_leg)
            phase_labels.extend(phase.label for phase in phases[1:])
            for relative_time, pose in sample_cycle(
                phases, self.phase_duration, self.sample_period
            ):
                positions = pose_to_joint_positions(
                    pose, self.hip_pitch_offset, self.arm_shoulder_pitch
                )
                largest_angle = max(largest_angle, *(abs(value) for value in positions))
                point = JointTrajectoryPoint()
                point.positions = positions
                point.time_from_start = self._duration(elapsed + relative_time)
                msg.points.append(point)
            elapsed += (len(phases) - 1) * self.phase_duration

        if largest_angle > self.max_abs_joint_rad:
            self.get_logger().error(
                f"Trajectory requires {largest_angle:.3f} rad, above "
                f"max_abs_joint_rad={self.max_abs_joint_rad:.3f}; not publishing"
            )
            return

        self.publisher.publish(msg)
        self.get_logger().warn(
            "Published OPEN-LOOP stair trajectory for simulation-first validation: "
            f"stairs={self.num_stairs}, height={self.step_height:.3f}m, "
            f"forward={self.step_forward:.3f}m, points={len(msg.points)}, "
            f"duration={elapsed:.1f}s, max_joint={largest_angle:.3f}rad"
        )
        self.get_logger().info("Phase order: " + " -> ".join(phase_labels))


def main(args=None):
    rclpy.init(args=args)
    node = StairClimberNode()
    try:
        rclpy.spin_once(node, timeout_sec=5.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
