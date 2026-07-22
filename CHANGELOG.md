# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2026-07-22

### Added

- Vendor-independent recording of `camera_bridge_core` color, depth, IMU,
  calibration, extrinsics, and timing measurements.
- ROS 2-compatible MCAP channels using CDR payloads and embedded `ros2msg`
  schemas without linking ROS, DDS, `rclcpp`, or `rosbag2`.
- Cross-platform `Ros2McapProducer` replay with real-time, accelerated, and
  unrestricted playback speeds.
- Stable numbered topic and frame naming for single-camera and multi-camera
  recordings.
- Optional Foxglove WebSocket live sink alongside authoritative MCAP writing.
- Windows and Linux CMake/Ninja build entry points and recorder round-trip
  tests.

[0.1.0]: https://github.com/wongkenchiang97/camera_bridge_mcap/releases/tag/v0.1.0
