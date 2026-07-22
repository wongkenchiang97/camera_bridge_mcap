#pragma once

#include <atomic>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include "camera_bridge_core/frame_events.hpp"

namespace camera_bridge_mcap {

class Ros2McapProducer : public bridge::IFrameProducer {
 public:
  struct Options {
    std::filesystem::path input_path;
    double replay_speed = 0.0;
  };

  explicit Ros2McapProducer(Options options);
  ~Ros2McapProducer();
  void setFrameConsumer(bridge::IFrameConsumer* consumer) override;
  void start() override;
  bool tryStart(std::string* error = nullptr);
  void stop() override;
  [[nodiscard]] bool running() const;
  [[nodiscard]] bridge::ProducerStats consumeStats() override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace camera_bridge_mcap
