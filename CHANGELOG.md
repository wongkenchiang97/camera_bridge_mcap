# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed

- Restored compatibility with the shared `camera_bridge_core` producer,
  stream-name, and frame-ID contracts required by recording and replay.

### Validation

- Linux build completed and both `camera_bridge_mcap_cdr_test` and
  `camera_bridge_mcap_roundtrip_test` passed.
- ROS 2 inspected `office_small_loop.mcap` as a valid 58.187815-second MCAP
  with 30,249 messages, including matched counts of 1,733 color and 1,733
  depth frames. The user also confirmed successful playback.

### Known Follow-up

- Retain exact replay completion status and terminal message counts in future
  runtime checkpoints.
- Validate the shared contract and recorder integration on MSVC after the
  Linux checkpoint.

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
