#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <mcap/reader.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>

#include "camera_bridge_mcap/ros2_messages.hpp"

namespace {
struct Stream {
  std::multiset<uint64_t> payload_stamps;
  std::multiset<uint64_t> timing_stamps;
  std::vector<std::pair<uint64_t, uint64_t>> device_stamps;
  uint64_t invalid_payloads = 0;
  uint64_t zero_device = 0;
  uint64_t mapped_capture = 0;
};

struct InfraredSample {
  uint64_t host_timestamp_us = 0;
  cv::Mat pixels;
};

cv::Matx33d rotationFromQuaternion(const std::array<double, 4>& q) {
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  const double norm = x*x + y*y + z*z + w*w;
  if (!std::isfinite(norm) || norm < 1.0e-12) return cv::Matx33d::zeros();
  const double s = 2.0 / norm;
  return cv::Matx33d(
      1.0-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w),
      s*(x*y+z*w), 1.0-s*(x*x+z*z), s*(y*z-x*w),
      s*(x*z-y*w), s*(y*z+x*w), 1.0-s*(x*x+y*y));
}

cv::Matx33d cameraMatrix(const camera_bridge_mcap::CameraInfoMessage& info) {
  return cv::Matx33d(info.k.data());
}

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const size_t index = static_cast<size_t>(
      std::ceil(fraction * static_cast<double>(values.size()))) - 1;
  return values[std::min(index, values.size()-1)];
}

void reportInfraredGeometry(
    const camera_bridge_mcap::CameraInfoMessage& left_info,
    const camera_bridge_mcap::CameraInfoMessage& right_info,
    const std::vector<camera_bridge_mcap::TransformStampedMessage>& transforms,
    const std::map<uint64_t, InfraredSample>& left_samples,
    const std::map<uint64_t, InfraredSample>& right_samples) {
  if (left_info.distortion_model != "plumb_bob" ||
      right_info.distortion_model != "plumb_bob" ||
      left_info.width != right_info.width ||
      left_info.height != right_info.height ||
      left_info.d.size() != 8 || right_info.d.size() != 8) {
    std::cout << "ir_geometry unavailable: unsupported calibration\n";
    return;
  }
  cv::Matx33d right_from_left_r;
  cv::Vec3d right_from_left_t;
  bool found_transform = false;
  for (const auto& transform : transforms) {
    const cv::Matx33d rotation = rotationFromQuaternion(transform.rotation);
    const cv::Vec3d translation(transform.translation.data());
    if (transform.header.frame_id == right_info.header.frame_id &&
        transform.child_frame_id == left_info.header.frame_id) {
      right_from_left_r = rotation;
      right_from_left_t = translation;
      found_transform = true;
      break;
    }
    if (transform.header.frame_id == left_info.header.frame_id &&
        transform.child_frame_id == right_info.header.frame_id) {
      right_from_left_r = rotation.t();
      right_from_left_t = -(rotation.t() * translation);
      found_transform = true;
      break;
    }
  }
  if (!found_transform || cv::norm(right_from_left_t) < 0.01) {
    std::cout << "ir_geometry unavailable: calibrated left/right baseline missing\n";
    return;
  }
  const cv::Size size(static_cast<int>(left_info.width),
                      static_cast<int>(left_info.height));
  const cv::Matx33d left_k = cameraMatrix(left_info);
  const cv::Matx33d right_k = cameraMatrix(right_info);
  const cv::Mat left_d(left_info.d), right_d(right_info.d);
  cv::Mat r1, r2, p1, p2, q;
  cv::stereoRectify(left_k, left_d, right_k, right_d, size,
                    right_from_left_r, right_from_left_t,
                    r1, r2, p1, p2, q, cv::CALIB_ZERO_DISPARITY, 0.0, size);
  std::cout << "ir_geometry baseline_m=" << cv::norm(right_from_left_t)
            << " sample_slots=" << left_samples.size() << "\n";
  const auto detector = cv::ORB::create(1500);
  const cv::BFMatcher matcher(cv::NORM_HAMMING);
  for (const auto& [index, left] : left_samples) {
    const auto right_it = right_samples.find(index);
    if (right_it == right_samples.end()) continue;
    const auto& right = right_it->second;
    const uint64_t skew = left.host_timestamp_us > right.host_timestamp_us
        ? left.host_timestamp_us - right.host_timestamp_us
        : right.host_timestamp_us - left.host_timestamp_us;
    if (skew > 1000) {
      std::cout << "ir_geometry frame=" << index << " skipped_skew_us="
                << skew << "\n";
      continue;
    }
    std::vector<cv::KeyPoint> left_keypoints, right_keypoints;
    cv::Mat left_descriptors, right_descriptors;
    detector->detectAndCompute(left.pixels, cv::noArray(),
                               left_keypoints, left_descriptors);
    detector->detectAndCompute(right.pixels, cv::noArray(),
                               right_keypoints, right_descriptors);
    if (left_descriptors.empty() || right_descriptors.empty()) continue;
    std::vector<std::vector<cv::DMatch>> candidate_matches;
    matcher.knnMatch(left_descriptors, right_descriptors,
                     candidate_matches, 2);
    std::vector<cv::Point2f> left_points, right_points;
    for (const auto& candidates : candidate_matches) {
      if (candidates.size() != 2 ||
          candidates[0].distance >= 0.75F * candidates[1].distance) continue;
      left_points.push_back(left_keypoints[candidates[0].queryIdx].pt);
      right_points.push_back(right_keypoints[candidates[0].trainIdx].pt);
    }
    if (left_points.size() < 8) {
      std::cout << "ir_geometry frame=" << index << " insufficient_matches="
                << left_points.size() << "\n";
      continue;
    }
    cv::Mat inlier_mask;
    cv::findFundamentalMat(left_points, right_points, cv::FM_RANSAC,
                           1.5, 0.99, inlier_mask);
    if (inlier_mask.empty()) continue;
    std::vector<cv::Point2f> rectified_left, rectified_right;
    cv::undistortPoints(left_points, rectified_left, left_k, left_d, r1, p1);
    cv::undistortPoints(right_points, rectified_right, right_k, right_d, r2, p2);
    std::vector<double> raw_y_error, rectified_y_error;
    for (size_t i = 0; i < left_points.size(); ++i) {
      if (!inlier_mask.at<uint8_t>(static_cast<int>(i))) continue;
      raw_y_error.push_back(std::abs(left_points[i].y-right_points[i].y));
      rectified_y_error.push_back(
          std::abs(rectified_left[i].y-rectified_right[i].y));
    }
    std::cout << "ir_geometry frame=" << index
              << " skew_us=" << skew
              << " ratio_matches=" << left_points.size()
              << " fundamental_inliers=" << raw_y_error.size()
              << " raw_y_p50_px=" << percentile(raw_y_error, 0.50)
              << " raw_y_p95_px=" << percentile(raw_y_error, 0.95)
              << " rectified_y_p50_px=" << percentile(rectified_y_error, 0.50)
              << " rectified_y_p95_px=" << percentile(rectified_y_error, 0.95)
              << "\n";
  }
}
}

int main(int argc, char** argv) {
  if (argc != 2 && argc != 3) {
    std::cerr << "Usage: camera_bridge_mcap_recording_audit <file.mcap> [unused-preview-prefix]\n";
    return 2;
  }
  using namespace camera_bridge_mcap;
  mcap::McapReader reader;
  const auto open = reader.open(argv[1]);
  if (!open.ok()) {
    std::cerr << open.message << "\n";
    return 2;
  }
  std::map<std::string, Stream> streams;
  std::map<uint8_t, std::string> image_kind{{1,"color"},{2,"depth"},
                                             {6,"infrared1"},{7,"infrared2"}};
  std::map<uint8_t, std::string> imu_kind{{3,"gyro"},{5,"accel"}};
  std::set<std::string> expected{"color","depth","infrared1","infrared2",
                                 "gyro","accel"};
  uint64_t camera_info = 0, tf_count = 0;
  std::map<std::string, uint64_t> image_index;
  std::optional<CameraInfoMessage> infrared1_info, infrared2_info;
  std::vector<TransformStampedMessage> transforms;
  std::map<uint64_t, InfraredSample> infrared1_samples, infrared2_samples;
  mcap::Timestamp previous_log_time = 0;
  uint64_t indexed_log_regressions = 0;
  for (const auto& view : reader.readMessages()) {
    indexed_log_regressions += view.message.logTime < previous_log_time;
    previous_log_time = view.message.logTime;
    const std::string& topic = view.channel->topic;
    const auto* bytes = reinterpret_cast<const uint8_t*>(view.message.data);
    const auto size = static_cast<size_t>(view.message.dataSize);
    try {
      if (topic == "/amr/camera_timing" ||
          topic == "/camera0/imu/gyro/device_time" ||
          topic == "/camera0/imu/accel/device_time") {
        const auto timing = decodeTiming(bytes, size);
        const auto& names = topic == "/amr/camera_timing" ? image_kind : imu_kind;
        const auto it = names.find(timing.stream_kind);
        if (it == names.end()) continue;
        auto& stream = streams[it->second];
        const auto stamp = timestampUs(timing.header.stamp);
        stream.timing_stamps.insert(stamp);
        stream.device_stamps.emplace_back(stamp, timing.device_timestamp_us);
        stream.zero_device += timing.device_timestamp_us == 0;
        stream.mapped_capture += timing.capture_steady_valid;
        stream.invalid_payloads += stamp != timing.host_timestamp_us;
      } else if (topic == "/camera0/color/image_raw" ||
                 topic == "/camera0/depth/image_raw" ||
                 topic == "/camera0/infrared1/image_raw" ||
                 topic == "/camera0/infrared2/image_raw") {
        const auto image = decodeImage(bytes, size);
        const std::string name = topic.find("infrared1") != std::string::npos ?
            "infrared1" : topic.find("infrared2") != std::string::npos ?
            "infrared2" : topic.find("depth") != std::string::npos ? "depth" : "color";
        auto& stream = streams[name];
        stream.payload_stamps.insert(timestampUs(image.header.stamp));
        if (name == "infrared1" || name == "infrared2") {
          const uint64_t index = ++image_index[name];
          if (index >= 120 && (index - 120) % 240 == 0 &&
              image.encoding == "mono8" && image.width > 0 &&
              image.height > 0 && image.step >= image.width &&
              image.data.size() == static_cast<size_t>(image.step) * image.height) {
            const cv::Mat pixels(static_cast<int>(image.height),
                                 static_cast<int>(image.width), CV_8UC1,
                                 const_cast<uint8_t*>(image.data.data()), image.step);
            auto& samples = name == "infrared1" ? infrared1_samples : infrared2_samples;
            samples.emplace(index, InfraredSample{
                timestampUs(image.header.stamp), pixels.clone()});
          }
        }
        if (argc == 3 && (name == "infrared1" || name == "infrared2") &&
            image_index[name] == 120) {
          const std::filesystem::path output = std::string(argv[2]) + "_" + name + ".png";
          if (std::filesystem::exists(output)) {
            std::cerr << "Refusing existing preview: " << output << "\n";
            return 4;
          }
          const cv::Mat pixels(static_cast<int>(image.height), static_cast<int>(image.width),
                               CV_8UC1, const_cast<uint8_t*>(image.data.data()), image.step);
          if (!cv::imwrite(output.string(), pixels)) return 4;
          std::cout << "preview " << output << "\n";
        }
        const std::string expected_encoding = name == "color" ? "bgr8" :
            name == "depth" ? "16UC1" : "mono8";
        stream.invalid_payloads += image.encoding != expected_encoding ||
            image.width == 0 || image.height == 0 ||
            image.data.size() != static_cast<size_t>(image.step) * image.height;
      } else if (topic == "/camera0/imu/gyro" || topic == "/camera0/imu/accel") {
        const auto imu = decodeImu(bytes, size);
        const std::string name = topic.find("gyro") != std::string::npos ? "gyro" : "accel";
        auto& stream = streams[name];
        stream.payload_stamps.insert(timestampUs(imu.header.stamp));
        for (const auto value : name == "gyro" ? imu.angular_velocity :
                                                     imu.linear_acceleration)
          stream.invalid_payloads += !std::isfinite(value);
      } else if (topic.find("/camera_info") != std::string::npos) {
        const auto info = decodeCameraInfo(bytes, size);
        ++camera_info;
        if (topic == "/camera0/infrared1/camera_info") infrared1_info = info;
        if (topic == "/camera0/infrared2/camera_info") infrared2_info = info;
        const bool has_distortion = std::any_of(
            info.d.begin(), info.d.end(),
            [](double coefficient) { return std::abs(coefficient) > 1.0e-12; });
        const bool has_rectification =
            info.r != std::array<double, 9>{{1, 0, 0, 0, 1, 0, 0, 0, 1}} ||
            info.p[3] != 0.0 || info.p[7] != 0.0;
        std::cout << "calibration " << topic << " " << info.width << "x" << info.height
                  << " fx=" << info.k[0] << " fy=" << info.k[4]
                  << " cx=" << info.k[2] << " cy=" << info.k[5]
                  << " distortion_model=" << info.distortion_model
                  << " distortion_nonzero=" << has_distortion
                  << " rectification_or_projection_offset=" << has_rectification
                  << " d=[";
        for (size_t index = 0; index < info.d.size(); ++index) {
          if (index != 0) std::cout << ',';
          std::cout << info.d[index];
        }
        std::cout << "]\n";
      } else if (topic == "/tf_static") {
        const auto tf = decodeTf(bytes, size);
        for (const auto& transform : tf.transforms) {
          ++tf_count;
          transforms.push_back(transform);
          std::cout << "tf " << transform.header.frame_id << " -> "
                    << transform.child_frame_id << " translation_m=("
                    << transform.translation[0] << "," << transform.translation[1] << ","
                    << transform.translation[2] << ")\n";
        }
      }
    } catch (const std::exception& error) {
      std::cerr << "Decode error on " << topic << ": " << error.what() << "\n";
      return 3;
    }
  }
  bool healthy = camera_info >= 4 && tf_count >= 1 && !indexed_log_regressions;
  std::cout << "camera_info=" << camera_info << " tf_transforms=" << tf_count
            << " indexed_log_regressions=" << indexed_log_regressions << "\n";
  for (const auto& name : expected) {
    auto& stream = streams[name];
    std::sort(stream.device_stamps.begin(), stream.device_stamps.end());
    uint64_t regressions = 0, duplicates = 0;
    for (size_t i = 1; i < stream.device_stamps.size(); ++i) {
      regressions += stream.device_stamps[i].second < stream.device_stamps[i-1].second;
      duplicates += stream.device_stamps[i].second == stream.device_stamps[i-1].second;
    }
    const bool paired = stream.payload_stamps == stream.timing_stamps;
    std::cout << name << " payloads=" << stream.payload_stamps.size()
              << " timing=" << stream.timing_stamps.size()
              << " paired=" << paired << " zero_device=" << stream.zero_device
              << " mapped_capture=" << stream.mapped_capture
              << " regressions=" << regressions << " duplicate_device=" << duplicates
              << " invalid=" << stream.invalid_payloads << "\n";
    healthy &= paired && !stream.payload_stamps.empty() && !stream.zero_device &&
               !regressions && !duplicates && !stream.invalid_payloads;
  }
  const auto& left = streams.at("infrared1").device_stamps;
  const auto& right = streams.at("infrared2").device_stamps;
  uint64_t max_host_delta_us = 0, max_device_delta_us = 0, matched = 0;
  size_t left_index = 0, right_index = 0;
  while (left_index < left.size() && right_index < right.size()) {
    const auto left_device = left[left_index].second;
    const auto right_device = right[right_index].second;
    const auto device_delta = left_device > right_device ?
        left_device - right_device : right_device - left_device;
    if (device_delta <= 1000) {
      const auto left_host = left[left_index].first;
      const auto right_host = right[right_index].first;
      const auto host_delta = left_host > right_host ?
          left_host - right_host : right_host - left_host;
      max_host_delta_us = std::max(max_host_delta_us, host_delta);
      max_device_delta_us = std::max(max_device_delta_us, device_delta);
      ++matched;
      ++left_index;
      ++right_index;
    } else if (left_device < right_device) {
      ++left_index;
    } else {
      ++right_index;
    }
  }
  std::cout << "ir_pairs=" << matched
            << " unmatched_ir1=" << left.size() - matched
            << " unmatched_ir2=" << right.size() - matched
            << " ir_pair_max_host_delta_us=" << max_host_delta_us
            << " ir_pair_max_device_delta_us=" << max_device_delta_us << "\n";
  if (infrared1_info && infrared2_info) {
    try {
      reportInfraredGeometry(*infrared1_info, *infrared2_info,
                             transforms, infrared1_samples, infrared2_samples);
    } catch (const cv::Exception& error) {
      std::cout << "ir_geometry unavailable: OpenCV error: "
                << error.what() << "\n";
    }
  }
  return healthy ? 0 : 1;
}
