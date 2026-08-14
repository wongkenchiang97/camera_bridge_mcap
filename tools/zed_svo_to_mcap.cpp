#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <sl/Camera.hpp>

#include "camera_bridge_core/stream_names.hpp"
#include "camera_bridge_mcap/recorder.hpp"

namespace {

uint64_t usFromNs(uint64_t timestamp_ns) {
  return timestamp_ns / 1000;
}

bridge::CameraIntrinsic intrinsicFromZed(
    const sl::CameraParameters& camera) {
  bridge::CameraIntrinsic output;
  output.width = static_cast<uint32_t>(camera.image_size.width);
  output.height = static_cast<uint32_t>(camera.image_size.height);
  output.fx = camera.fx;
  output.fy = camera.fy;
  output.cx = camera.cx;
  output.cy = camera.cy;
  return output;
}

cv::Mat bgrFromZed(sl::Mat& image) {
  if (image.getDataType() != sl::MAT_TYPE::U8_C4 ||
      image.getMemoryType() != sl::MEM::CPU) {
    throw std::runtime_error("ZED image is not CPU U8_C4");
  }
  cv::Mat bgra(
      static_cast<int>(image.getHeight()),
      static_cast<int>(image.getWidth()),
      CV_8UC4,
      image.getPtr<sl::uchar1>(sl::MEM::CPU),
      image.getStepBytes(sl::MEM::CPU));
  cv::Mat bgr;
  cv::cvtColor(bgra, bgr, cv::COLOR_BGRA2BGR);
  return bgr;
}

cv::Mat depthU16FromZed(sl::Mat& depth) {
  if (depth.getDataType() != sl::MAT_TYPE::U16_C1 ||
      depth.getMemoryType() != sl::MEM::CPU) {
    throw std::runtime_error("ZED depth is not CPU U16_C1");
  }
  return cv::Mat(
      static_cast<int>(depth.getHeight()),
      static_cast<int>(depth.getWidth()),
      CV_16UC1,
      depth.getPtr<sl::ushort1>(sl::MEM::CPU),
      depth.getStepBytes(sl::MEM::CPU)).clone();
}

sl::DEPTH_MODE parseDepthMode(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });
  if (value == "none") return sl::DEPTH_MODE::NONE;
  if (value == "performance") return sl::DEPTH_MODE::PERFORMANCE;
  if (value == "quality") return sl::DEPTH_MODE::QUALITY;
  if (value == "ultra") return sl::DEPTH_MODE::ULTRA;
  if (value == "neural_light") return sl::DEPTH_MODE::NEURAL_LIGHT;
  if (value == "neural") return sl::DEPTH_MODE::NEURAL;
  if (value == "neural_plus") return sl::DEPTH_MODE::NEURAL_PLUS;
  throw std::invalid_argument(
      "depth_mode must be none, performance, quality, ultra, "
      "neural_light, neural, or neural_plus");
}

void emitCalibration(
    camera_bridge_mcap::Ros2McapRecorder& recorder,
    uint32_t source_id,
    uint64_t timestamp_us,
    const sl::CalibrationParameters& calibration,
    const sl::SensorsConfiguration& sensors,
    bool depth_enabled) {
  bridge::CameraCalibrationEvent event;
  event.source_id = source_id;
  event.timestamp_us = timestamp_us;
  event.has_color = true;
  event.has_right = true;
  event.has_depth = depth_enabled;
  event.color_intrinsic = intrinsicFromZed(calibration.left_cam);
  event.right_intrinsic = intrinsicFromZed(calibration.right_cam);
  if (depth_enabled) {
    // DEPTH_U16_MM is registered to the rectified left image.
    event.depth_intrinsic = event.color_intrinsic;
  }
  event.color_frame_id = bridge::cameraColorOpticalFrame(source_id);
  event.right_frame_id = bridge::cameraRightOpticalFrame(source_id);
  if (depth_enabled) {
    event.depth_frame_id = bridge::cameraDepthOpticalFrame(source_id);
  }
  recorder.onCameraCalibration(event);

  bridge::ExtrinsicsEvent extrinsics;
  extrinsics.source_id = source_id;
  extrinsics.timestamp_us = timestamp_us;
  bridge::ExtrinsicTransformEvent transform;
  // ZED's stereo_transform describes the right-camera pose in the left-camera
  // frame. Publish that direction directly as T_left_right. Consumers that
  // require T_right_left can invert it; labeling this as right->left reverses
  // the baseline sign and places positive-disparity points behind the cameras.
  transform.parent_frame_id = event.color_frame_id;
  transform.child_frame_id = event.right_frame_id;
  const sl::Rotation rotation =
      calibration.stereo_transform.getRotationMatrix();
  const sl::Translation translation =
      calibration.stereo_transform.getTranslation();
  for (int index = 0; index < 9; ++index) {
    transform.extrinsic.rot[index] = rotation.r[index];
  }
  transform.extrinsic.trans[0] = translation.x;
  transform.extrinsic.trans[1] = translation.y;
  transform.extrinsic.trans[2] = translation.z;
  transform.extrinsic.translation_scale_to_meters = 1.0;
  extrinsics.transforms.push_back(transform);
  if (depth_enabled) {
    bridge::ExtrinsicTransformEvent depth_transform;
    depth_transform.parent_frame_id = event.color_frame_id;
    depth_transform.child_frame_id = event.depth_frame_id;
    depth_transform.extrinsic.rot[0] = 1.0F;
    depth_transform.extrinsic.rot[4] = 1.0F;
    depth_transform.extrinsic.rot[8] = 1.0F;
    depth_transform.extrinsic.translation_scale_to_meters = 1.0;
    extrinsics.transforms.push_back(depth_transform);
  }
  bridge::ExtrinsicTransformEvent imu_transform;
  imu_transform.parent_frame_id = event.color_frame_id;
  imu_transform.child_frame_id = bridge::cameraImuFrame(source_id);
  const sl::Rotation camera_from_imu_rotation =
      sensors.camera_imu_transform.getRotationMatrix();
  const sl::Translation camera_from_imu_translation =
      sensors.camera_imu_transform.getTranslation();
  for (int index = 0; index < 9; ++index) {
    imu_transform.extrinsic.rot[index] = camera_from_imu_rotation.r[index];
  }
  imu_transform.extrinsic.trans[0] = camera_from_imu_translation.x;
  imu_transform.extrinsic.trans[1] = camera_from_imu_translation.y;
  imu_transform.extrinsic.trans[2] = camera_from_imu_translation.z;
  imu_transform.extrinsic.translation_scale_to_meters = 1.0;
  extrinsics.transforms.push_back(imu_transform);
  recorder.onExtrinsics(extrinsics);
}

void emitImuBatch(
    camera_bridge_mcap::Ros2McapRecorder& recorder,
    uint32_t source_id,
    const std::vector<sl::SensorsData>& batch,
    uint64_t& last_imu_timestamp_us) {
  constexpr float kRadiansPerDegree =
      3.14159265358979323846F / 180.0F;
  for (const auto& sensors : batch) {
    if (!sensors.imu.is_available) {
      continue;
    }
    const uint64_t timestamp_us =
        usFromNs(sensors.imu.timestamp.getNanoseconds());
    if (timestamp_us == 0 || timestamp_us <= last_imu_timestamp_us) {
      continue;
    }
    bridge::ImuSampleEvent event;
    event.source_id = source_id;
    event.timestamp_us = timestamp_us;
    event.device_timestamp_us = timestamp_us;
    event.frame_id = bridge::cameraImuFrame(source_id);
    event.has_accel = true;
    event.accel = {
        sensors.imu.linear_acceleration.x,
        sensors.imu.linear_acceleration.y,
        sensors.imu.linear_acceleration.z};
    event.has_gyro = true;
    event.gyro = {
        sensors.imu.angular_velocity.x * kRadiansPerDegree,
        sensors.imu.angular_velocity.y * kRadiansPerDegree,
        sensors.imu.angular_velocity.z * kRadiansPerDegree};
    if (last_imu_timestamp_us != 0) {
      event.dt_sec = static_cast<double>(
          timestamp_us - last_imu_timestamp_us) * 1.0e-6;
      event.dt_valid = event.dt_sec > 0.0;
    }
    recorder.onImuSample(event);
    last_imu_timestamp_us = timestamp_us;
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3 || argc > 5) {
    std::cerr << "Usage: camera_bridge_zed_svo_to_mcap "
                 "<input.svo2> <output.mcap> [source_id] [depth_mode]\n";
    return 2;
  }
  const std::filesystem::path input_path(argv[1]);
  const std::filesystem::path output_path(argv[2]);
  const uint32_t source_id = argc >= 4
      ? static_cast<uint32_t>(std::stoul(argv[3]))
      : 0;
  sl::DEPTH_MODE depth_mode = sl::DEPTH_MODE::NONE;
  try {
    if (argc == 5) {
      depth_mode = parseDepthMode(argv[4]);
    }
  } catch (const std::exception& e) {
    std::cerr << "Invalid depth mode: " << e.what() << "\n";
    return 2;
  }
  const bool depth_enabled = depth_mode != sl::DEPTH_MODE::NONE;
  if (!std::filesystem::is_regular_file(input_path)) {
    std::cerr << "Input SVO2 does not exist: " << input_path << "\n";
    return 2;
  }
  if (std::filesystem::exists(output_path)) {
    std::cerr << "Refusing to overwrite existing output: " << output_path
              << "\n";
    return 2;
  }

  sl::Camera camera;
  sl::InitParameters init;
  init.input.setFromSVOFile(input_path.string().c_str());
  init.depth_mode = depth_mode;
  init.camera_disable_self_calib = true;
  init.coordinate_units = sl::UNIT::METER;
  init.sdk_verbose = 1;
  const sl::ERROR_CODE open_status = camera.open(init);
  if (open_status != sl::ERROR_CODE::SUCCESS) {
    std::cerr << "ZED SVO open failed: " << sl::toString(open_status) << "\n";
    return 3;
  }

  camera_bridge_mcap::Ros2McapRecorder::Options recorder_options;
  recorder_options.output_path = output_path;
  recorder_options.use_zstd = true;
  camera_bridge_mcap::Ros2McapRecorder recorder(recorder_options);
  std::string error;
  if (!recorder.start(&error)) {
    std::cerr << "MCAP recorder start failed: " << error << "\n";
    camera.close();
    return 4;
  }

  const sl::CameraInformation information = camera.getCameraInformation();
  const auto& calibration =
      information.camera_configuration.calibration_parameters;
  sl::Mat left;
  sl::Mat right;
  sl::Mat depth;
  uint64_t frames = 0;
  uint64_t imu_samples = 0;
  uint64_t last_imu_timestamp_us = 0;
  bool calibration_emitted = false;
  for (;;) {
    const sl::ERROR_CODE status = camera.grab();
    if (status == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
      break;
    }
    if (status != sl::ERROR_CODE::SUCCESS) {
      std::cerr << "ZED grab failed at frame " << frames << ": "
                << sl::toString(status) << "\n";
      recorder.stop();
      camera.close();
      return 5;
    }
    if (camera.retrieveImage(left, sl::VIEW::LEFT, sl::MEM::CPU) !=
            sl::ERROR_CODE::SUCCESS ||
        camera.retrieveImage(right, sl::VIEW::RIGHT, sl::MEM::CPU) !=
            sl::ERROR_CODE::SUCCESS) {
      std::cerr << "ZED rectified image retrieval failed at frame "
                << frames << "\n";
      recorder.stop();
      camera.close();
      return 5;
    }
    const uint64_t timestamp_us = usFromNs(
        camera.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds());
    if (!calibration_emitted) {
      emitCalibration(
          recorder,
          source_id,
          timestamp_us,
          calibration,
          information.sensors_configuration,
          depth_enabled);
      calibration_emitted = true;
    }
    bridge::ColorFrameEvent left_event;
    left_event.source_id = source_id;
    left_event.timestamp_us = timestamp_us;
    left_event.device_timestamp_us = timestamp_us;
    left_event.frame_id = bridge::cameraColorOpticalFrame(source_id);
    left_event.bgr = bgrFromZed(left);
    recorder.onColorFrame(left_event);
    bridge::RightFrameEvent right_event;
    right_event.source_id = source_id;
    right_event.timestamp_us = timestamp_us;
    right_event.device_timestamp_us = timestamp_us;
    right_event.frame_id = bridge::cameraRightOpticalFrame(source_id);
    right_event.bgr = bgrFromZed(right);
    recorder.onRightFrame(right_event);
    if (depth_enabled) {
      if (camera.retrieveMeasure(
              depth, sl::MEASURE::DEPTH_U16_MM, sl::MEM::CPU) !=
          sl::ERROR_CODE::SUCCESS) {
        std::cerr << "ZED registered depth retrieval failed at frame "
                  << frames << "\n";
        recorder.stop();
        camera.close();
        return 5;
      }
      bridge::DepthFrameEvent depth_event;
      depth_event.source_id = source_id;
      depth_event.timestamp_us = timestamp_us;
      depth_event.device_timestamp_us = timestamp_us;
      depth_event.frame_id = bridge::cameraDepthOpticalFrame(source_id);
      depth_event.depth_mono16 = depthU16FromZed(depth);
      recorder.onDepthFrame(depth_event);
    }

    std::vector<sl::SensorsData> sensor_batch;
    if (camera.getSensorsDataBatch(sensor_batch) == sl::ERROR_CODE::SUCCESS) {
      const uint64_t before = last_imu_timestamp_us;
      emitImuBatch(
          recorder, source_id, sensor_batch, last_imu_timestamp_us);
      if (last_imu_timestamp_us != before) {
        ++imu_samples;
      }
    }
    ++frames;
    if (frames % 100 == 0) {
      std::cout << "Converted " << frames << "/"
                << camera.getSVONumberOfFrames() << " frames\r"
                << std::flush;
    }
  }
  recorder.stop();
  camera.close();
  std::cout << "\nConverted frames=" << frames
            << " imu_batches_with_data=" << imu_samples
            << " depth_mode=" << sl::toString(depth_mode)
            << " serial=" << information.serial_number
            << " output=" << output_path << "\n";
  return frames == 0 ? 6 : 0;
}
