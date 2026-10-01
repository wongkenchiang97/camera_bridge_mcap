#include <chrono>
#include <filesystem>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <thread>
#include <mcap/reader.hpp>

#include "camera_bridge_mcap/recorder.hpp"
#include "camera_bridge_mcap/producer.hpp"
#include "camera_bridge_mcap/ros2_messages.hpp"

struct Sink final : bridge::IFrameConsumer {
  int gyro=0,accel=0,secondCamera=0,infrared=0,derived=0,parameters=0,secondExtrinsics=0;
  void onColorFrame(const bridge::ColorFrameEvent&) override {}
  void onRightFrame(const bridge::RightFrameEvent&) override {}
  void onDepthFrame(const bridge::DepthFrameEvent&) override {}
  void onImuSample(const bridge::ImuSampleEvent& e) override {
    if(e.has_gyro&&!e.has_accel&&e.device_timestamp_us==111)++gyro;
    if(e.has_accel&&!e.has_gyro&&e.device_timestamp_us==222)++accel;
    if(e.source_id==1&&e.has_gyro&&e.device_timestamp_us==333)++secondCamera;
  }
  void onExtrinsics(const bridge::ExtrinsicsEvent& e) override {
    if(e.source_id==1&&e.transforms.size()==1)++secondExtrinsics;
  }
  void onCameraCalibration(const bridge::CameraCalibrationEvent&) override {}
  void onInfraredFrame(const bridge::InfraredFrameEvent& e) override {
    if(e.sensor_index==1&&e.device_timestamp_us==444&&e.mono8.cols==2&&
       e.mono8.at<uint8_t>(0,0)==17)++infrared;
  }
  void onDerivedImuSample(const bridge::ImuSampleEvent& e) override {
    if(e.has_gyro&&e.has_accel&&e.device_timestamp_us==555)++derived;
  }
  void onCameraParameters(const bridge::CameraParametersEvent& e) override {
    if(e.serial_number=="serial0"&&e.sdk_version=="2.56.5"&&
       e.values.size()==1&&e.values[0].effective=="1")++parameters;
  }
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
  bridge::CameraParametersEvent parameters;
  parameters.source_id=0;parameters.timestamp_us=1000004;
  parameters.serial_number="serial0";parameters.sdk_version="2.56.5";
  parameters.values.push_back({"depth_sensor.inter_cam_sync_mode","1","1","override"});
  recorder.onCameraParameters(parameters);
  bridge::InfraredFrameEvent infrared;
  infrared.source_id=0;infrared.sensor_index=1;
  // Simulate a late callback carrying an earlier exact capture timestamp.
  infrared.timestamp_us=999005;infrared.device_timestamp_us=444;
  infrared.mono8=cv::Mat(1,2,CV_8UC1);infrared.mono8.at<uint8_t>(0,0)=17;
  infrared.mono8.at<uint8_t>(0,1)=18;
  recorder.onInfraredFrame(infrared);
  bridge::ImuSampleEvent combined=gyro;
  combined.timestamp_us=1000006;combined.device_timestamp_us=555;
  combined.has_accel=true;combined.accel={7,8,9};
  recorder.onDerivedImuSample(combined);
  bridge::ExtrinsicsEvent extrinsics;
  extrinsics.source_id=1;
  extrinsics.timestamp_us=1000007;
  bridge::ExtrinsicTransformEvent transform;
  transform.parent_frame_id="camera1_color_optical_frame";
  transform.child_frame_id="camera1_depth_optical_frame";
  extrinsics.transforms.push_back(transform);
  recorder.onExtrinsics(extrinsics);
  recorder.stop();

  mcap::McapReader reader;
  if(!reader.open(path.string()).ok())return 2;
  std::map<std::string,uint64_t> payloadStamp,timingStamp;
  std::set<mcap::ChannelId> channelsWithMessages;
  mcap::Timestamp previousLogTime=0;
  for(const auto& view:reader.readMessages()) {
    if(view.message.logTime<previousLogTime)return 17;
    previousLogTime=view.message.logTime;
    channelsWithMessages.insert(view.channel->id);
    const auto& topic=view.channel->topic;
    if(topic=="/camera0/recorder/parameters") {
      const auto& message=view.message;
      const auto decoded=decodeDiagnosticArray(
          reinterpret_cast<const uint8_t*>(message.data),message.dataSize);
      if(decoded.status.size()!=1||decoded.status[0].hardware_id!="serial0")return 12;
      payloadStamp[topic]=timestampUs(decoded.header.stamp);
    }
    const auto* bytes=reinterpret_cast<const uint8_t*>(view.message.data);
    const auto size=static_cast<size_t>(view.message.dataSize);
    if(topic=="/camera0/infrared1/image_raw") {
      const auto decoded=decodeImage(bytes,size);
      if(decoded.encoding!="mono8"||decoded.data.size()!=2||decoded.data[0]!=17)return 13;
      if(view.message.publishTime!=999005000)return 18;
      payloadStamp[topic]=timestampUs(decoded.header.stamp);
    }
    if(topic=="/camera0/imu/data") {
      const auto decoded=decodeImu(bytes,size);
      if(decoded.linear_acceleration[0]!=7)return 15;
      payloadStamp[topic]=timestampUs(decoded.header.stamp);
    }
    if(topic=="/camera0/imu/gyro"||topic=="/camera0/imu/accel"||
       topic=="/camera1/imu/gyro")
      payloadStamp[topic]=timestampUs(decodeImu(bytes,size).header.stamp);
    if(topic=="/camera0/imu/gyro/device_time"||topic=="/camera0/imu/accel/device_time"||
       topic=="/camera1/imu/gyro/device_time"||
       topic=="/camera0/imu/data/device_time"||
       topic=="/amr/camera_timing") {
      const auto timing=decodeTiming(bytes,size);
      if(topic=="/amr/camera_timing"&&timing.stream_kind==6) {
        if(timing.device_timestamp_us!=444)return 14;
        timingStamp["/camera0/infrared1/device_time"]=timestampUs(timing.header.stamp);
        continue;
      }
      if(topic=="/amr/camera_timing")continue;
      const uint64_t expected=timing.stream_kind==8?555u:
          timing.source_id==1?333u:(timing.stream_kind==3?111u:222u);
      if(timing.device_timestamp_us!=expected)return 3;
      timingStamp[topic]=timestampUs(timing.header.stamp);
    }
  }
  for(const auto& channel:reader.channels()) {
    if(!channelsWithMessages.count(channel.first))return 16;
  }
  reader.close();
  if(payloadStamp.size()!=6||payloadStamp["/camera0/recorder/parameters"]!=1000004||
     payloadStamp["/camera0/infrared1/image_raw"]!=timingStamp["/camera0/infrared1/device_time"]||timingStamp.size()!=5||
     payloadStamp["/camera0/imu/data"]!=timingStamp["/camera0/imu/data/device_time"]||
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
  if(sink.gyro!=1||sink.accel!=1||sink.secondCamera!=1||sink.infrared!=1||sink.derived!=1||sink.parameters!=1||sink.secondExtrinsics!=1)return 6;
  std::error_code ec;
  fs::remove(path,ec);
  return 0;
}
