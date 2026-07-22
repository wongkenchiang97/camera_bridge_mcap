# camera_bridge_mcap

Cross-platform, ROS-runtime-free recording and deterministic replay of
`camera_bridge_core` measurements in ROS 2 compatible MCAP files.

The library does **not** link ROS, DDS, `rclcpp`, or `rosbag2`. It writes MCAP
channels with `cdr` message encoding and embedded `ros2msg` schemas so a ROS 2
Humble installation can inspect and play the standard sensor topics.

## Initial topic contract

| Topic | Type |
|---|---|
| `/camera{source_id}/color/image_raw` | `sensor_msgs/msg/Image` (`bgr8`) |
| `/camera{source_id}/depth/image_raw` | `sensor_msgs/msg/Image` (`16UC1`) |
| `/camera{source_id}/color/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/camera{source_id}/depth/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/camera{source_id}/imu/data_raw` | `sensor_msgs/msg/Imu` |
| `/tf_static` | `tf2_msgs/msg/TFMessage` |
| `/amr/camera_timing` | `camera_bridge_msgs/msg/FrameTiming` |

`header.stamp` is the measurement time. MCAP publish/log time uses the bridge
host timestamp. `/amr/camera_timing` preserves source ID, device time, steady
clock mapping, timestamp source, and uncertainty.

Frame IDs are always numbered, including single-camera recordings:
`camera0_color_optical_frame`, `camera0_depth_optical_frame`, and
`camera0_imu_frame`. This keeps one-camera and multi-camera datasets compatible.

## Build

```powershell
cmake -S . -B build-ninja-msvc -G Ninja `
  -DCMAKE_TOOLCHAIN_FILE=../vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build-ninja-msvc
ctest --test-dir build-ninja-msvc --output-on-failure
```

## Optional Foxglove live sink

The recorder can expose the same ROS 2 CDR messages to Foxglove over a
WebSocket while writing the MCAP file. This remains vendor-independent: it
receives only `camera_bridge_core` events and has no Orbbec or RealSense SDK
dependency. MCAP recording is authoritative; live publication is disabled by
default and a live-server failure does not stop recording unless
`live_publish_required` is enabled.

Build the optional sink against a Foxglove SDK checkout or binary package:

```powershell
cmake -S . -B build-ninja-msvc-foxglove -G Ninja `
  -DCMAKE_TOOLCHAIN_FILE=../vcpkg/scripts/buildsystems/vcpkg.cmake `
  -DCAMERA_BRIDGE_MCAP_WITH_FOXGLOVE=ON `
  -DFOXGLOVE_SDK_ROOT=../foxglove-sdk
cmake --build build-ninja-msvc-foxglove
ctest --test-dir build-ninja-msvc-foxglove --output-on-failure
```

At runtime set `Ros2McapRecorder::Options::live_publish_enabled = true`.
The default endpoint is `ws://127.0.0.1:8765`; host and port are configurable.
Foxglove sees the same numbered camera topics and embedded `ros2msg` schemas
that are written to MCAP.

## Status

The first milestone records and replays raw color, raw depth, IMU, calibration,
static extrinsics, and bridge timing. Rosbag2 `metadata.yaml`, segmentation,
lossless-compressed mapping profiles, CLI tools, and vSLAM selection flags are
the next integration steps.

Channels are created lazily for every observed `source_id`, so one MCAP can
contain independently calibrated cameras with different resolutions without
interleaving them on one ROS topic.
