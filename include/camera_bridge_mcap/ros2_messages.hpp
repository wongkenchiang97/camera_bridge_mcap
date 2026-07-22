#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "camera_bridge_core/frame_events.hpp"

namespace camera_bridge_mcap {

struct RosTime { int32_t sec = 0; uint32_t nanosec = 0; };
struct Header { RosTime stamp; std::string frame_id; };

struct ImageMessage {
  Header header;
  uint32_t height = 0;
  uint32_t width = 0;
  std::string encoding;
  bool is_bigendian = false;
  uint32_t step = 0;
  std::vector<uint8_t> data;
};

struct ImuMessage {
  Header header;
  std::array<double, 4> orientation{{0.0, 0.0, 0.0, 1.0}};
  std::array<double, 9> orientation_covariance{{-1.0}};
  std::array<double, 3> angular_velocity{};
  std::array<double, 9> angular_velocity_covariance{};
  std::array<double, 3> linear_acceleration{};
  std::array<double, 9> linear_acceleration_covariance{};
};

struct CameraInfoMessage {
  Header header;
  uint32_t height = 0;
  uint32_t width = 0;
  std::string distortion_model;
  std::vector<double> d;
  std::array<double, 9> k{};
  std::array<double, 9> r{{1,0,0,0,1,0,0,0,1}};
  std::array<double, 12> p{};
  uint32_t binning_x = 0;
  uint32_t binning_y = 0;
  uint32_t roi_x_offset = 0;
  uint32_t roi_y_offset = 0;
  uint32_t roi_height = 0;
  uint32_t roi_width = 0;
  bool roi_do_rectify = false;
};

struct TimingMessage {
  Header header;
  uint32_t source_id = 0;
  uint8_t stream_kind = 0;
  uint64_t host_timestamp_us = 0;
  uint64_t device_timestamp_us = 0;
  uint64_t driver_receive_steady_us = 0;
  uint64_t capture_steady_us = 0;
  bool capture_steady_valid = false;
  uint8_t timestamp_source = 0;
  double clock_mapping_uncertainty_us = 0.0;
};

struct TransformStampedMessage {
  Header header;
  std::string child_frame_id;
  std::array<double, 3> translation{};
  std::array<double, 4> rotation{{0.0, 0.0, 0.0, 1.0}};
};
struct TfMessage { std::vector<TransformStampedMessage> transforms; };

std::vector<uint8_t> encode(const ImageMessage& message);
std::vector<uint8_t> encode(const ImuMessage& message);
std::vector<uint8_t> encode(const CameraInfoMessage& message);
std::vector<uint8_t> encode(const TimingMessage& message);
std::vector<uint8_t> encode(const TfMessage& message);
ImageMessage decodeImage(const uint8_t* data, size_t size);
ImuMessage decodeImu(const uint8_t* data, size_t size);
CameraInfoMessage decodeCameraInfo(const uint8_t* data, size_t size);
TimingMessage decodeTiming(const uint8_t* data, size_t size);
TfMessage decodeTf(const uint8_t* data, size_t size);

RosTime rosTimeFromUs(uint64_t timestamp_us);
uint64_t timestampUs(const RosTime& time);
std::string distortionModelName(bridge::CameraDistortionModel model);

const std::string& imageSchema();
const std::string& imuSchema();
const std::string& cameraInfoSchema();
const std::string& timingSchema();
const std::string& tfSchema();

}  // namespace camera_bridge_mcap
