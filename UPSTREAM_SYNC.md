# Upstream ver6 sync

- Source: `https://github.com/Do-ng-woo/biped_bike_robot.git`
- Source commit: `9e69a7a203f6ac8969e612879a4dd39355e524bd`
- Source date: 2026-08-23
- Integrated date: 2026-08-25
- SolidWorks export: `biped_bike_robot_print_ver6`

The upstream repository is used as a read-only reference. This package keeps the
independent ROS package name `biped_bike_runtime`, so it can coexist with the
original `biped_bike_robot` package.

## Integrated

- ver6 URDF and STL meshes
- Dynamixel IDs 1 through 19, including wrist pitch and wrist roll
- updated transform/revert sequences and ready posture
- hardware telemetry, present joint-state publishing, and OpenCR IMU support
- IK walker, web controller, OpenCR firmware examples, and updated tests

## Runtime-specific additions retained

- Gazebo lidar and ground-truth odometry
- wheel command conversion and odometry
- maze/stairs worlds, Nav2 launch files, saved maps, and footprint measurement
- stable `/dev/serial/by-id` paths for OpenCR and RPLIDAR

## Hardware safety notes

- `hardware_display.launch.py` starts with motor torque and startup movement off.
- Motor ID 19 is configured but its physical direction is still marked unchecked.
- Verify all 19 IDs and ID 19 direction at low speed with the robot supported
  before enabling the ver6 transform sequence.
