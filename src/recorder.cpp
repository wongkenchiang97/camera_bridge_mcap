#include "camera_bridge_mcap/recorder.hpp"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <cmath>
#include <map>
#include <mcap/writer.hpp>
#include "camera_bridge_mcap/ros2_messages.hpp"
#include "camera_bridge_core/stream_names.hpp"
#include "live_sink.hpp"

namespace camera_bridge_mcap { namespace {
struct Out { mcap::Schema schema; mcap::Channel channel; std::string schemaText; uint32_t sequence=0; bool registered=false; };
Out makeChannel(const char* topic,const char* type,const std::string& text){Out o;o.schema.name=type;o.schema.encoding="ros2msg";o.schemaText=text;o.schema.data.resize(text.size());std::memcpy(o.schema.data.data(),text.data(),text.size());o.channel.topic=topic;o.channel.messageEncoding="cdr";o.channel.metadata["offered_qos_profiles"]="[]";return o;}
std::vector<uint8_t> bytes(const cv::Mat&m){cv::Mat c=m.isContinuous()?m:m.clone();return{c.data,c.data+c.total()*c.elemSize()};}
TimingMessage timing(uint8_t kind,uint32_t source,uint64_t host,uint64_t device,const bridge::FrameTiming&t,const std::string&frame){TimingMessage m;m.header={rosTimeFromUs(host),frame};m.source_id=source;m.stream_kind=kind;m.host_timestamp_us=host;m.device_timestamp_us=device;m.driver_receive_steady_us=t.driver_receive_steady_us;m.capture_steady_us=t.capture_steady_us;m.capture_steady_valid=t.capture_steady_valid;m.timestamp_source=static_cast<uint8_t>(t.timestamp_source);m.clock_mapping_uncertainty_us=t.clock_mapping_uncertainty_us;return m;}
CameraInfoMessage info(const bridge::CameraIntrinsic&i,const bridge::CameraDistortion&d,uint64_t ts,const std::string&frame){CameraInfoMessage m;m.header={rosTimeFromUs(ts),frame};m.width=i.width;m.height=i.height;m.distortion_model=distortionModelName(d.model);m.d={d.k1,d.k2,d.p1,d.p2,d.k3,d.k4,d.k5,d.k6};m.k={i.fx,0,i.cx,0,i.fy,i.cy,0,0,1};m.p={i.fx,0,i.cx,0,0,i.fy,i.cy,0,0,0,1,0};return m;}
std::array<double,4> quaternion(const float*r){std::array<double,4>q{};const double trace=r[0]+r[4]+r[8];if(trace>0){const double s=std::sqrt(trace+1.0)*2;q={(r[7]-r[5])/s,(r[2]-r[6])/s,(r[3]-r[1])/s,.25*s};}else if(r[0]>r[4]&&r[0]>r[8]){const double s=std::sqrt(1.0+r[0]-r[4]-r[8])*2;q={.25*s,(r[1]+r[3])/s,(r[2]+r[6])/s,(r[7]-r[5])/s};}else if(r[4]>r[8]){const double s=std::sqrt(1.0+r[4]-r[0]-r[8])*2;q={(r[1]+r[3])/s,.25*s,(r[5]+r[7])/s,(r[2]-r[6])/s};}else{const double s=std::sqrt(1.0+r[8]-r[0]-r[4])*2;q={(r[2]+r[6])/s,(r[5]+r[7])/s,.25*s,(r[3]-r[1])/s};}return q;}
}
class Ros2McapRecorder::Impl { public:
 enum class Stream {Color,Right,Depth,Imu,ColorInfo,RightInfo,DepthInfo,Parameters,Infrared1,Infrared2,Infrared1Info,Infrared2Info,DerivedImu};
 struct SourceChannels {
  Out color,right,depth,imu,colorInfo,rightInfo,depthInfo;
  Out gyro,accel,gyroTime,accelTime;
  Out parameters;
  Out infrared1,infrared2,infrared1Info,infrared2Info;
  Out derivedImu,derivedImuTime;
  explicit SourceChannels(const std::string& prefix)
      :color(makeChannel((prefix+"/color/image_raw").c_str(),"sensor_msgs/msg/Image",imageSchema())),
       right(makeChannel((prefix+"/right/image_raw").c_str(),"sensor_msgs/msg/Image",imageSchema())),
       depth(makeChannel((prefix+"/depth/image_raw").c_str(),"sensor_msgs/msg/Image",imageSchema())),
       imu(makeChannel((prefix+"/imu/data_raw").c_str(),"sensor_msgs/msg/Imu",imuSchema())),
       colorInfo(makeChannel((prefix+"/color/camera_info").c_str(),"sensor_msgs/msg/CameraInfo",cameraInfoSchema())),
       rightInfo(makeChannel((prefix+"/right/camera_info").c_str(),"sensor_msgs/msg/CameraInfo",cameraInfoSchema())),
       depthInfo(makeChannel((prefix+"/depth/camera_info").c_str(),"sensor_msgs/msg/CameraInfo",cameraInfoSchema())),
       gyro(makeChannel((prefix+"/imu/gyro").c_str(),"sensor_msgs/msg/Imu",imuSchema())),
       accel(makeChannel((prefix+"/imu/accel").c_str(),"sensor_msgs/msg/Imu",imuSchema())),
       gyroTime(makeChannel((prefix+"/imu/gyro/device_time").c_str(),"camera_bridge_msgs/msg/FrameTiming",timingSchema())),
       accelTime(makeChannel((prefix+"/imu/accel/device_time").c_str(),"camera_bridge_msgs/msg/FrameTiming",timingSchema())),
       parameters(makeChannel((prefix+"/recorder/parameters").c_str(),"diagnostic_msgs/msg/DiagnosticArray",diagnosticArraySchema())),
       infrared1(makeChannel((prefix+"/infrared1/image_raw").c_str(),"sensor_msgs/msg/Image",imageSchema())),
       infrared2(makeChannel((prefix+"/infrared2/image_raw").c_str(),"sensor_msgs/msg/Image",imageSchema())),
       infrared1Info(makeChannel((prefix+"/infrared1/camera_info").c_str(),"sensor_msgs/msg/CameraInfo",cameraInfoSchema())),
       infrared2Info(makeChannel((prefix+"/infrared2/camera_info").c_str(),"sensor_msgs/msg/CameraInfo",cameraInfoSchema())),
       derivedImu(makeChannel((prefix+"/imu/data").c_str(),"sensor_msgs/msg/Imu",imuSchema())),
       derivedImuTime(makeChannel((prefix+"/imu/data/device_time").c_str(),"camera_bridge_msgs/msg/FrameTiming",timingSchema())) {}
 };
 explicit Impl(Options o):options(std::move(o)),timingCh(makeChannel("/amr/camera_timing","camera_bridge_msgs/msg/FrameTiming",timingSchema())),tfStatic(makeChannel("/tf_static","tf2_msgs/msg/TFMessage",tfSchema())){}
 void add(Out& c) {
  if(!options.write_mcap||c.registered)return;
  writer.addSchema(c.schema);
  c.channel.schemaId=c.schema.id;
  writer.addChannel(c.channel);
  c.registered=true;
 }
 bool open(std::string* error) {
  std::lock_guard<std::mutex> lock(mu);
  if(active)return true;
  if(!options.write_mcap&&!options.live_publish_enabled) {
   if(error)*error="preview-only mode requires live publishing";
   return false;
  }
  if(options.write_mcap) {
   std::error_code ec;
   if(!options.output_path.parent_path().empty())
    std::filesystem::create_directories(options.output_path.parent_path(),ec);
   if(ec){if(error)*error=ec.message();return false;}
   mcap::McapWriterOptions writerOptions("ros2");
   writerOptions.chunkSize=options.chunk_size_bytes;
   writerOptions.compression=options.use_zstd?mcap::Compression::Zstd:mcap::Compression::None;
   const auto status=writer.open(options.output_path.string(),writerOptions);
   if(!status.ok()){if(error)*error=status.message;return false;}
   lastLogTimeNs=0;
  }
  if(options.live_publish_enabled) {
   live=createLiveSink();
   std::string liveError;
   if(!live->start(options.live_publish_host,options.live_publish_port,liveError)) {
    live.reset();
    if(options.live_publish_required||!options.write_mcap) {
     if(options.write_mcap)writer.close();
     if(error)*error="failed to start Foxglove live sink: "+liveError;
     return false;
    }
    lastLiveError=liveError;
   }
  }
  active=true;
  return true;
 }
 void close() {
  std::lock_guard<std::mutex> lock(mu);
  if(live){live->stop();live.reset();}
  if(active&&options.write_mcap)writer.close();
  active=false;
  sources.clear();
  timingCh.registered=false;
  tfStatic.registered=false;
 }
 void emit(Out& c,const std::vector<uint8_t>& data,uint64_t us) {
  if(options.write_mcap) {
   add(c);
   mcap::Message message;
   message.channelId=c.channel.id;
   message.sequence=c.sequence++;
   const auto nowNs=static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
       std::chrono::system_clock::now().time_since_epoch()).count());
   message.logTime=std::max(nowNs,lastLogTimeNs+1);
   lastLogTimeNs=message.logTime;
   message.publishTime=us*1000;
   message.data=reinterpret_cast<const std::byte*>(data.data());
   message.dataSize=data.size();
   writer.write(message);
  }
  if(live) {
   std::string liveError;
   if(!live->publish(c.channel.topic,c.schema.name,c.schemaText,data,us*1000,liveError))
    lastLiveError=liveError;
  }
 }
 SourceChannels& source(uint32_t id){
  auto name=options.camera_namespaces.find(id);
  const std::string prefix=name==options.camera_namespaces.end()?bridge::cameraTopicPrefix(id):name->second;
  auto [it,inserted]=sources.try_emplace(id,prefix);
  (void)inserted;
  return it->second;
 }
 void writeRawImu(uint32_t id,bool gyro,const std::vector<uint8_t>&timingData,
                  const std::vector<uint8_t>&data,uint64_t us){
  std::lock_guard<std::mutex>l(mu);if(!active)return;auto&s=source(id);
  Out& payload=gyro?s.gyro:s.accel;
  if(options.write_timing_metadata)emit(gyro?s.gyroTime:s.accelTime,timingData,us);
  emit(payload,data,us);
 }
 Out& channel(SourceChannels&s,Stream stream){switch(stream){case Stream::Color:return s.color;case Stream::Right:return s.right;case Stream::Depth:return s.depth;case Stream::Imu:return s.imu;case Stream::ColorInfo:return s.colorInfo;case Stream::RightInfo:return s.rightInfo;case Stream::DepthInfo:return s.depthInfo;case Stream::Parameters:return s.parameters;case Stream::Infrared1:return s.infrared1;case Stream::Infrared2:return s.infrared2;case Stream::Infrared1Info:return s.infrared1Info;case Stream::Infrared2Info:return s.infrared2Info;case Stream::DerivedImu:return s.derivedImu;}return s.color;}
 void writeDerivedImu(uint32_t id,const std::vector<uint8_t>&timingData,const std::vector<uint8_t>&data,uint64_t us){std::lock_guard<std::mutex>l(mu);if(!active)return;auto&s=source(id);emit(s.derivedImuTime,timingData,us);emit(s.derivedImu,data,us);}
 void write(uint32_t id,Stream stream,const std::vector<uint8_t>&data,uint64_t us){std::lock_guard<std::mutex>l(mu);if(!active)return;auto&s=source(id);emit(channel(s,stream),data,us);}
 void writeTimed(uint32_t id,Stream stream,const std::vector<uint8_t>&timingData,const std::vector<uint8_t>&data,uint64_t us){std::lock_guard<std::mutex>l(mu);if(!active)return;auto&s=source(id);emit(timingCh,timingData,us);emit(channel(s,stream),data,us);}
 void writeGlobal(Out&c,const std::vector<uint8_t>&data,uint64_t us){std::lock_guard<std::mutex>l(mu);if(active)emit(c,data,us);}
 Options options;mutable std::mutex mu;mcap::McapWriter writer;bool active=false;uint64_t lastLogTimeNs=0;std::map<uint32_t,SourceChannels>sources;Out timingCh,tfStatic;std::unique_ptr<LiveSink>live;std::string lastLiveError;
};
Ros2McapRecorder::Ros2McapRecorder(Options o):impl_(std::make_unique<Impl>(std::move(o))){} Ros2McapRecorder::~Ros2McapRecorder(){stop();}
bool Ros2McapRecorder::start(std::string*e){return impl_->open(e);}void Ros2McapRecorder::stop(){impl_->close();}bool Ros2McapRecorder::running()const{std::lock_guard<std::mutex>l(impl_->mu);return impl_->active;}
void Ros2McapRecorder::onColorFrame(const bridge::ColorFrameEvent&e){const auto frame=bridge::cameraColorOpticalFrame(e.source_id);ImageMessage m;m.header={rosTimeFromUs(e.timestamp_us),frame};m.height=e.bgr.rows;m.width=e.bgr.cols;m.encoding="bgr8";m.step=e.bgr.cols*3;m.data=bytes(e.bgr);const auto data=encode(m);if(impl_->options.write_timing_metadata)impl_->writeTimed(e.source_id,Impl::Stream::Color,encode(timing(1,e.source_id,e.timestamp_us,e.device_timestamp_us,e.timing,frame)),data,e.timestamp_us);else impl_->write(e.source_id,Impl::Stream::Color,data,e.timestamp_us);}
void Ros2McapRecorder::onRightFrame(const bridge::RightFrameEvent&e){const auto frame=bridge::cameraRightOpticalFrame(e.source_id);ImageMessage m;m.header={rosTimeFromUs(e.timestamp_us),frame};m.height=e.bgr.rows;m.width=e.bgr.cols;m.encoding="bgr8";m.step=e.bgr.cols*3;m.data=bytes(e.bgr);const auto data=encode(m);if(impl_->options.write_timing_metadata)impl_->writeTimed(e.source_id,Impl::Stream::Right,encode(timing(4,e.source_id,e.timestamp_us,e.device_timestamp_us,e.timing,frame)),data,e.timestamp_us);else impl_->write(e.source_id,Impl::Stream::Right,data,e.timestamp_us);}
void Ros2McapRecorder::onDepthFrame(const bridge::DepthFrameEvent&e){const auto frame=bridge::cameraDepthOpticalFrame(e.source_id);ImageMessage m;m.header={rosTimeFromUs(e.timestamp_us),frame};m.height=e.depth_mono16.rows;m.width=e.depth_mono16.cols;m.encoding="16UC1";m.step=e.depth_mono16.cols*2;m.data=bytes(e.depth_mono16);const auto data=encode(m);if(impl_->options.write_timing_metadata)impl_->writeTimed(e.source_id,Impl::Stream::Depth,encode(timing(2,e.source_id,e.timestamp_us,e.device_timestamp_us,e.timing,frame)),data,e.timestamp_us);else impl_->write(e.source_id,Impl::Stream::Depth,data,e.timestamp_us);}
void Ros2McapRecorder::onInfraredFrame(const bridge::InfraredFrameEvent&e){if(e.sensor_index!=1&&e.sensor_index!=2)return;const auto frame=bridge::cameraInfraredOpticalFrame(e.source_id,e.sensor_index);ImageMessage m;m.header={rosTimeFromUs(e.timestamp_us),frame};m.height=e.mono8.rows;m.width=e.mono8.cols;m.encoding="mono8";m.step=e.mono8.cols;m.data=bytes(e.mono8);const auto data=encode(m);const auto stream=e.sensor_index==1?Impl::Stream::Infrared1:Impl::Stream::Infrared2;if(impl_->options.write_timing_metadata)impl_->writeTimed(e.source_id,stream,encode(timing(e.sensor_index==1?6:7,e.source_id,e.timestamp_us,e.device_timestamp_us,e.timing,frame)),data,e.timestamp_us);else impl_->write(e.source_id,stream,data,e.timestamp_us);}
void Ros2McapRecorder::onImuSample(const bridge::ImuSampleEvent&e){
 const auto frame=e.frame_id.empty()?bridge::cameraImuFrame(e.source_id):e.frame_id;
 ImuMessage m;m.header={rosTimeFromUs(e.timestamp_us),frame};
 if(e.has_gyro)m.angular_velocity={e.gyro.x,e.gyro.y,e.gyro.z};
 if(e.has_accel)m.linear_acceleration={e.accel.x,e.accel.y,e.accel.z};
 const auto data=encode(m);
 if(impl_->options.split_raw_imu){
  // An input event is one physical motion sample, never an interpolated pair.
  if(e.has_gyro==e.has_accel)return;
  impl_->writeRawImu(e.source_id,e.has_gyro,
      encode(timing(e.has_gyro?3:5,e.source_id,e.timestamp_us,e.device_timestamp_us,e.timing,frame)),data,e.timestamp_us);
 }else if(impl_->options.write_timing_metadata)
  impl_->writeTimed(e.source_id,Impl::Stream::Imu,encode(timing(3,e.source_id,e.timestamp_us,e.device_timestamp_us,e.timing,frame)),data,e.timestamp_us);
 else impl_->write(e.source_id,Impl::Stream::Imu,data,e.timestamp_us);
}
void Ros2McapRecorder::onDerivedImuSample(const bridge::ImuSampleEvent&e){if(!e.has_gyro||!e.has_accel||e.device_timestamp_us==0)return;const auto frame=e.frame_id.empty()?bridge::cameraImuFrame(e.source_id):e.frame_id;ImuMessage m;m.header={rosTimeFromUs(e.timestamp_us),frame};m.angular_velocity={e.gyro.x,e.gyro.y,e.gyro.z};m.linear_acceleration={e.accel.x,e.accel.y,e.accel.z};impl_->writeDerivedImu(e.source_id,encode(timing(8,e.source_id,e.timestamp_us,e.device_timestamp_us,e.timing,frame)),encode(m),e.timestamp_us);}
void Ros2McapRecorder::onExtrinsics(const bridge::ExtrinsicsEvent&e){TfMessage m;for(const auto&x:e.transforms){TransformStampedMessage t;t.header={rosTimeFromUs(e.timestamp_us),x.parent_frame_id};t.child_frame_id=x.child_frame_id;const double scale=x.extrinsic.translation_scale_to_meters;t.translation={x.extrinsic.trans[0]*scale,x.extrinsic.trans[1]*scale,x.extrinsic.trans[2]*scale};t.rotation=quaternion(x.extrinsic.rot);m.transforms.push_back(std::move(t));}impl_->writeGlobal(impl_->tfStatic,encode(m),e.timestamp_us);}
void Ros2McapRecorder::onCameraCalibration(const bridge::CameraCalibrationEvent&e){if(e.has_color)impl_->write(e.source_id,Impl::Stream::ColorInfo,encode(info(e.color_intrinsic,e.color_distortion,e.timestamp_us,bridge::cameraColorOpticalFrame(e.source_id))),e.timestamp_us);if(e.has_right)impl_->write(e.source_id,Impl::Stream::RightInfo,encode(info(e.right_intrinsic,e.right_distortion,e.timestamp_us,bridge::cameraRightOpticalFrame(e.source_id))),e.timestamp_us);if(e.has_depth)impl_->write(e.source_id,Impl::Stream::DepthInfo,encode(info(e.depth_intrinsic,e.depth_distortion,e.timestamp_us,bridge::cameraDepthOpticalFrame(e.source_id))),e.timestamp_us);if(e.has_infrared1)impl_->write(e.source_id,Impl::Stream::Infrared1Info,encode(info(e.infrared1_intrinsic,e.infrared1_distortion,e.timestamp_us,bridge::cameraInfraredOpticalFrame(e.source_id,1))),e.timestamp_us);if(e.has_infrared2)impl_->write(e.source_id,Impl::Stream::Infrared2Info,encode(info(e.infrared2_intrinsic,e.infrared2_distortion,e.timestamp_us,bridge::cameraInfraredOpticalFrame(e.source_id,2))),e.timestamp_us);}
void Ros2McapRecorder::onCameraParameters(const bridge::CameraParametersEvent&e){DiagnosticArrayMessage m;m.header={rosTimeFromUs(e.timestamp_us),bridge::cameraColorOpticalFrame(e.source_id)};DiagnosticStatusMessage s;s.name="/camera"+std::to_string(e.source_id)+"/recorder/parameters";s.message="effective RealSense settings";s.hardware_id=e.serial_number;s.values.push_back({"sdk_version",e.sdk_version});for(const auto&v:e.values){s.values.push_back({v.name+".requested",v.requested});s.values.push_back({v.name+".effective",v.effective});s.values.push_back({v.name+".origin",v.origin});}m.status.push_back(std::move(s));impl_->write(e.source_id,Impl::Stream::Parameters,encode(m),e.timestamp_us);}
} // namespace camera_bridge_mcap
