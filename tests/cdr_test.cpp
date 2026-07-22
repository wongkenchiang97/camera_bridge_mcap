#include <cmath>
#include <iostream>
#include "camera_bridge_mcap/ros2_messages.hpp"

int main(){using namespace camera_bridge_mcap;
 ImageMessage image;image.header={rosTimeFromUs(1234567),"camera"};image.height=2;image.width=3;image.encoding="bgr8";image.step=9;image.data={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};auto ib=encode(image);auto ir=decodeImage(ib.data(),ib.size());if(timestampUs(ir.header.stamp)!=1234567||ir.header.frame_id!="camera"||ir.data!=image.data||ir.step!=9)return 1;
 ImuMessage imu;imu.header={rosTimeFromUs(9000001),"imu"};imu.angular_velocity={1.25,-2.5,3.75};imu.linear_acceleration={4,5,6};auto mb=encode(imu);auto mr=decodeImu(mb.data(),mb.size());if(mr.angular_velocity!=imu.angular_velocity||mr.linear_acceleration!=imu.linear_acceleration)return 2;
 CameraInfoMessage ci;ci.header={rosTimeFromUs(10),"optical"};ci.width=640;ci.height=480;ci.distortion_model="plumb_bob";ci.d={.1,.2,.3,.4,.5};ci.k={500,0,320,0,501,240,0,0,1};auto cb=encode(ci);auto cr=decodeCameraInfo(cb.data(),cb.size());if(cr.k!=ci.k||cr.d!=ci.d||cr.width!=640)return 3;
 TimingMessage t;t.header={rosTimeFromUs(77),"camera"};t.source_id=4;t.device_timestamp_us=55;t.capture_steady_valid=true;t.clock_mapping_uncertainty_us=2.5;auto tb=encode(t);auto tr=decodeTiming(tb.data(),tb.size());if(tr.source_id!=4||tr.device_timestamp_us!=55||!tr.capture_steady_valid||std::abs(tr.clock_mapping_uncertainty_us-2.5)>1e-12)return 4;
 TfMessage tf;TransformStampedMessage transform;transform.header={rosTimeFromUs(88),"base"};transform.child_frame_id="camera";transform.translation={1,2,3};transform.rotation={0,0,0,1};tf.transforms.push_back(transform);auto fb=encode(tf);auto fr=decodeTf(fb.data(),fb.size());if(fr.transforms.size()!=1||fr.transforms[0].translation!=transform.translation)return 5;
 std::cout<<"CDR contracts passed\n";return 0;}
