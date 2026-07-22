#pragma once

#include <filesystem>
#include <memory>
#include <mutex>
#include <string>

#include "camera_bridge_core/frame_events.hpp"

namespace camera_bridge_mcap {

class Ros2McapRecorder final : public bridge::IFrameConsumer {
 public:
  struct Options {
    std::filesystem::path output_path;
    std::string color_frame_id = "camera_color_optical_frame";
    std::string depth_frame_id = "camera_depth_optical_frame";
    std::string imu_frame_id = "camera_imu_frame";
    bool write_timing_metadata = true;
    bool use_zstd = true;
    uint64_t chunk_size_bytes = 4 * 1024 * 1024;
    bool live_publish_enabled = false;
    std::string live_publish_host = "127.0.0.1";
    uint16_t live_publish_port = 8765;
    bool live_publish_required = false;
  };

  explicit Ros2McapRecorder(Options options);
  ~Ros2McapRecorder() override;
  bool start(std::string* error = nullptr);
  void stop();
  [[nodiscard]] bool running() const;

  void onColorFrame(const bridge::ColorFrameEvent& event) override;
  void onDepthFrame(const bridge::DepthFrameEvent& event) override;
  void onImuSample(const bridge::ImuSampleEvent& event) override;
  void onExtrinsics(const bridge::ExtrinsicsEvent& event) override;
  void onCameraCalibration(const bridge::CameraCalibrationEvent& event) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace camera_bridge_mcap
