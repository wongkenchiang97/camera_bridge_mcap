#include <chrono>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <mcap/reader.hpp>

#include "camera_bridge_mcap/recorder.hpp"
#include "camera_bridge_mcap/producer.hpp"
#include "camera_bridge_mcap/ros2_messages.hpp"

struct Sink final : bridge::IFrameConsumer {
  int gyro=0,accel=0,secondCamera=0;
  void onColorFrame(const bridge::ColorFrameEvent&) override {}
  void onRightFrame(const bridge::RightFrameEvent&) override {}
  void onDepthFrame(const bridge::DepthFrameEvent&) override {}
  void onImuSample(const bridge::ImuSampleEvent& e) override {
    if(e.has_gyro&&!e.has_accel&&e.device_timestamp_us==111)++gyro;
    if(e.has_accel&&!e.has_gyro&&e.device_timestamp_us==222)++accel;
    if(e.source_id==1&&e.has_gyro&&e.device_timestamp_us==333)++secondCamera;
  }
  void onExtrinsics(const bridge::ExtrinsicsEvent&) override {}
  void onCameraCalibration(const bridge::CameraCalibrationEvent&) override {}
};

int main() {
  namespace fs=std::filesystem;
  using namespace camera_bridge_mcap;
  const auto unique=std::chrono::steady_clock::now().time_since_epoch().count();
  const auto path=fs::temp_directory_path()/
      ("camera_bridge_split_imu_"+std::to_string(unique)+".mcap");
  Ros2McapRecorder::Options options;
  options.output_path=path;
  options.split_raw_imu=true;
  options.use_zstd=false;
  options.camera_namespaces[0]="/camera0";
  options.camera_namespaces[1]="/camera1";
  Ros2McapRecorder recorder(options);
  std::string error;
  if(!recorder.start(&error)) {std::cerr<<error;return 1;}
  bridge::ImuSampleEvent gyro;
  gyro.source_id=0;gyro.timestamp_us=1000001;gyro.device_timestamp_us=111;
  gyro.has_gyro=true;gyro.gyro={1,2,3};gyro.frame_id="camera0_imu_frame";
  recorder.onImuSample(gyro);
  bridge::ImuSampleEvent accel=gyro;
  accel.timestamp_us=1000002;accel.device_timestamp_us=222;
  accel.has_gyro=false;accel.has_accel=true;accel.accel={4,5,6};
  recorder.onImuSample(accel);
  bridge::ImuSampleEvent second=gyro;
  second.source_id=1;second.timestamp_us=1000003;
  second.device_timestamp_us=333;second.frame_id="camera1_imu_frame";
  recorder.onImuSample(second);
  recorder.stop();

  mcap::McapReader reader;
  if(!reader.open(path.string()).ok())return 2;
  std::map<std::string,uint64_t> payloadStamp,timingStamp;
  for(const auto& view:reader.readMessages()) {
    const auto& topic=view.channel->topic;
    const auto* bytes=reinterpret_cast<const uint8_t*>(view.message.data);
    const auto size=static_cast<size_t>(view.message.dataSize);
    if(topic=="/camera0/imu/gyro"||topic=="/camera0/imu/accel"||
       topic=="/camera1/imu/gyro")
      payloadStamp[topic]=timestampUs(decodeImu(bytes,size).header.stamp);
    if(topic=="/camera0/imu/gyro/device_time"||topic=="/camera0/imu/accel/device_time"||
       topic=="/camera1/imu/gyro/device_time") {
      const auto timing=decodeTiming(bytes,size);
      const uint64_t expected=timing.source_id==1?333u:(timing.stream_kind==3?111u:222u);
      if(timing.device_timestamp_us!=expected)return 3;
      timingStamp[topic]=timestampUs(timing.header.stamp);
    }
  }
  reader.close();
  if(payloadStamp.size()!=3||timingStamp.size()!=3||
     payloadStamp["/camera0/imu/gyro"]!=timingStamp["/camera0/imu/gyro/device_time"]||
     payloadStamp["/camera0/imu/accel"]!=timingStamp["/camera0/imu/accel/device_time"]||
     payloadStamp["/camera1/imu/gyro"]!=timingStamp["/camera1/imu/gyro/device_time"])
    return 4;
  Sink sink;
  Ros2McapProducer::Options playback;
  playback.input_path=path;
  Ros2McapProducer producer(playback);
  producer.setFrameConsumer(&sink);
  if(!producer.tryStart(&error)) {std::cerr<<error;return 5;}
  for(int i=0;i<200&&producer.running();++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  producer.stop();
  if(sink.gyro!=1||sink.accel!=1||sink.secondCamera!=1)return 6;
  std::error_code ec;
  fs::remove(path,ec);
  return 0;
}
