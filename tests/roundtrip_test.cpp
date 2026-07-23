#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <thread>
#include "camera_bridge_mcap/producer.hpp"
#include "camera_bridge_mcap/recorder.hpp"
#include "camera_bridge_core/stream_names.hpp"

class Sink final:public bridge::IFrameConsumer{public:
 void onColorFrame(const bridge::ColorFrameEvent&e)override{color=e;colorSources.push_back(e.source_id);colorDeviceTimestamps.push_back(e.device_timestamp_us);colors++;}
 void onDepthFrame(const bridge::DepthFrameEvent&e)override{depth=e;depthDeviceTimestamps.push_back(e.device_timestamp_us);depths++;}
 void onImuSample(const bridge::ImuSampleEvent&e)override{imu=e;imuDeviceTimestamps.push_back(e.device_timestamp_us);imus++;}
 void onExtrinsics(const bridge::ExtrinsicsEvent&e)override{extrinsics=e;extrinsicEvents++;}
 void onCameraCalibration(const bridge::CameraCalibrationEvent&e)override{calibration=e;calibrations++;}
 bridge::ColorFrameEvent color;bridge::DepthFrameEvent depth;bridge::ImuSampleEvent imu;bridge::CameraCalibrationEvent calibration;bridge::ExtrinsicsEvent extrinsics;std::vector<uint32_t>colorSources;std::vector<uint64_t>colorDeviceTimestamps,depthDeviceTimestamps,imuDeviceTimestamps;int colors=0,depths=0,imus=0,calibrations=0,extrinsicEvents=0;
};
int main(){namespace fs=std::filesystem;using namespace camera_bridge_mcap;auto path=fs::temp_directory_path()/"camera_bridge_mcap_roundtrip.mcap";std::error_code ec;fs::remove(path,ec);
 Ros2McapRecorder::Options ro;ro.output_path=path;ro.use_zstd=false;
#ifdef CAMERA_BRIDGE_MCAP_TEST_LIVE_SINK
 ro.live_publish_enabled=true;ro.live_publish_required=true;ro.live_publish_port=0;
#endif
 Ros2McapRecorder recorder(ro);std::string error;if(!recorder.start(&error)){std::cerr<<error;return 1;}
 bridge::CameraCalibrationEvent cal;cal.source_id=7;cal.timestamp_us=1000000;cal.has_color=true;cal.color_intrinsic={2,2,100,101,1,1};recorder.onCameraCalibration(cal);
 bridge::ExtrinsicsEvent ext;ext.timestamp_us=1000050;bridge::ExtrinsicTransformEvent tx;tx.parent_frame_id="base";tx.child_frame_id="camera";tx.extrinsic.trans[0]=100;tx.extrinsic.translation_scale_to_meters=.001;ext.transforms.push_back(tx);recorder.onExtrinsics(ext);
 bridge::ColorFrameEvent color;color.source_id=7;color.timestamp_us=1000100;color.device_timestamp_us=555;color.bgr=cv::Mat(2,2,CV_8UC3);for(int i=0;i<12;++i)color.bgr.data[i]=static_cast<uint8_t>(i+1);color.timing.capture_steady_valid=true;color.timing.capture_steady_us=999;recorder.onColorFrame(color);
 bridge::ColorFrameEvent color2=color;color2.source_id=8;color2.timestamp_us=1000150;color2.device_timestamp_us=655;recorder.onColorFrame(color2);
 bridge::DepthFrameEvent depth;depth.source_id=7;depth.timestamp_us=1000200;depth.device_timestamp_us=556;depth.depth_mono16=cv::Mat(2,2,CV_16UC1);depth.depth_mono16.at<uint16_t>(0,0)=1234;depth.depth_mono16.at<uint16_t>(0,1)=2345;depth.depth_mono16.at<uint16_t>(1,0)=3456;depth.depth_mono16.at<uint16_t>(1,1)=4567;
 bridge::ImuSampleEvent imu;imu.source_id=7;imu.timestamp_us=1000250;imu.device_timestamp_us=557;imu.has_accel=imu.has_gyro=true;imu.accel={1,2,3};imu.gyro={4,5,6};
 // Record later data first so a log-time ordered reader separates timing
 // records from their sensor messages. Matching must not depend on adjacency.
 recorder.onImuSample(imu);recorder.onDepthFrame(depth);recorder.stop();
 Sink sink;Ros2McapProducer::Options po;po.input_path=path;Ros2McapProducer producer(po);producer.setFrameConsumer(&sink);if(!producer.tryStart(&error)){std::cerr<<error;return 2;}for(int i=0;i<200&&producer.running();++i)std::this_thread::sleep_for(std::chrono::milliseconds(5));producer.stop();
 if(sink.calibration.source_id!=7||sink.calibration.color_frame_id!="camera7_color_optical_frame")return 9;if(sink.colors!=2||sink.depths!=1||sink.imus!=1||sink.calibrations!=1||sink.extrinsicEvents!=1)return 3;if(sink.colorSources!=std::vector<uint32_t>({7,8})||sink.colorDeviceTimestamps!=std::vector<uint64_t>({555,655})||sink.depthDeviceTimestamps!=std::vector<uint64_t>({556})||sink.imuDeviceTimestamps!=std::vector<uint64_t>({557})||sink.color.frame_id!=bridge::cameraColorOpticalFrame(8))return 4;if(cv::norm(sink.color.bgr,color.bgr,cv::NORM_INF)!=0||cv::norm(sink.depth.depth_mono16,depth.depth_mono16,cv::NORM_INF)!=0)return 5;if(sink.imu.accel.x!=1||sink.imu.gyro.z!=6||sink.imu.frame_id!="camera7_imu_frame")return 6;if(sink.extrinsics.transforms.size()!=1||std::abs(sink.extrinsics.transforms[0].extrinsic.trans[0]-.1f)>1e-6f)return 7;if(bridge::cameraTopicPrefix(12)!="/camera12"||bridge::cameraDepthOpticalFrame(12)!="camera12_depth_optical_frame")return 8;fs::remove(path,ec);std::cout<<"MCAP round trip passed\n";return 0;}
