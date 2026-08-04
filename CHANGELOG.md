# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Added ROS 2 MCAP recording and replay for vendor-neutral right-camera images
  and calibration on `/cameraN/right/image_raw` and
  `/cameraN/right/camera_info`, including independent timing metadata and
  producer counters. Depth remains a separate optional stream.
- Added an optional ZED-SDK SVO2 converter for deterministic rectified stereo
  MCAP generation. It disables self-calibration, depth, and tracking; exports
  resolved rectified intrinsics, stereo and IMU extrinsics, synchronized BGR
  images, and recorded IMU; and refuses to overwrite existing output.

### Fixed

- Corrected the ZED converter's stereo TF direction so the SDK-provided right
  camera pose is published as `T_left_right`. Replay inversion now yields the
  negative `T_right_left` baseline required by positive rectified disparity,
  instead of triangulating every observation behind the cameras.
- Accumulated per-source color/right/depth camera-info messages before
  dispatch so downstream stereo consumers receive one coherent calibration
  state instead of losing color calibration when right camera info arrives.
  Replay now also restores all eight supported distortion coefficients.
- Restored compatibility with the shared `camera_bridge_core` producer,
  stream-name, and frame-ID contracts required by recording and replay.

### Validation

- Linux build completed and both `camera_bridge_mcap_cdr_test` and
  `camera_bridge_mcap_roundtrip_test` passed.
- The ZED converter target built and ran against SDK 5.4/CUDA 12.2 on the RTX
  4060 host. It converted all 1,502 pairs and 5,895 IMU samples from
  `stereo_office_small_loop.svo2` into the structurally verified
  50.045395-second `stereo_office_small_loop_rectified_r2.mcap` without vendor
  depth. The immutable output is 10,271,386,791 bytes with SHA-256
  `5870076d39f4f368b33b1eecc92e360c8b776f951fcc5191f517a94be81e1f84`.
- Full-fidelity vSLAM stereo-shadow R14 replayed the corrected MCAP at `1.0x`, reached clean
  EOF with exit code zero, and accounted for all 1,502 stereo pairs as 1,493
  processed plus nine explicit downstream policy drops.
- ROS 2 inspected `office_small_loop.mcap` as a valid 58.187815-second MCAP
  with 30,249 messages, including matched counts of 1,733 color and 1,733
  depth frames. The user also confirmed successful playback.

### Known Follow-up

- Propagate recorder write/close failures explicitly; the upstream release
  MCAP file writer can ignore short `fwrite()` results in release builds, as
  exposed by the intentionally discarded disk-full conversion attempt.
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
