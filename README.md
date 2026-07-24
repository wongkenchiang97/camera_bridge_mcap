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

Replay correlates timing metadata by `(source_id, stream_kind,
host_timestamp_us)` rather than assuming a timing record is adjacent to its
sensor payload. This keeps existing recordings valid when concurrent color,
depth, and IMU callbacks interleaved their MCAP records. New recordings write
each timing record and sensor payload under one recorder lock so the pair is
atomic. If an exact timing record is unavailable, replay retains the sensor
header timestamp as the device-time fallback.

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

### 2026-07-23 checkpoint

The first milestone records and replays raw color, raw depth, IMU, calibration,
static extrinsics, and bridge timing. MSVC builds pass with both CDR and
round-trip tests. Full-fidelity replay of the 1.23 GiB
`mapping_room.mcap` recording through VSLAM reached EOF at approximately
30.9 Hz color/depth and 203.8 Hz IMU. The timing-correlation fix reduced
aggregated RGB-D ingress drops from 3238 in the pre-fix run to 11, enabled
784 visual frames and 78 runtime VIO commits, and produced persistent mapping
nodes. The remaining 11 depth drops also occur in repeated corrected runs and
belong to downstream VSLAM scheduling rather than MCAP timestamp association.

Channels are created lazily for every observed `source_id`, so one MCAP can
contain independently calibrated cameras with different resolutions without
interleaving them on one ROS topic.

### 2026-07-24 synchronized Orbbec checkpoint

The Linux library build and both CDR and round-trip tests pass against the
shared `camera_bridge_core` producer, numbered stream-name, and frame-ID
contracts. The Orbbec recorder produced
`recordings/office_small_loop.mcap`, and ROS 2 inspection reported:

- duration: 58.187815 seconds;
- size: 1.6 GiB;
- total messages: 30,249;
- color/depth images: 1,733 each;
- IMU messages: 11,657;
- color/depth camera info: one each;
- static TF messages: one;
- timing messages: 15,123;
- SHA-256:
  `06e02db9cc9c670fe7a5c772783e71d4cdb91f2e240c4f26a948e485d13aeec4`.

Playback was observed working. Exact replay completion status and terminal
per-stream counters were not retained, so deterministic replay accounting
remains open.

Next work:

1. Add a deterministic legacy-record regression that explicitly writes
   interleaved timing and sensor records instead of relying only on the
   recorder's now-atomic output.
2. Expose producer completion status, decode errors, and terminal message
   counts so EOF is distinguishable from an asynchronous decode failure.
3. Bound or stream the replay timing index for multi-hour, large-scale mapping
   recordings.
4. Add rosbag2 `metadata.yaml`, segmentation, mapping profiles, and standalone
   recorder/player CLI tools as product requirements mature.
