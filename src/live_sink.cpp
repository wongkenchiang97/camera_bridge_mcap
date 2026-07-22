#include "live_sink.hpp"

#ifndef CAMERA_BRIDGE_MCAP_WITH_FOXGLOVE
namespace camera_bridge_mcap { namespace {
class UnavailableLiveSink final:public LiveSink { public:
 bool start(const std::string&,uint16_t,std::string&error)override{error="camera_bridge_mcap was built without Foxglove live support";return false;}
 void stop()override{}
 bool publish(const std::string&,const std::string&,const std::string&,const std::vector<uint8_t>&,uint64_t,std::string&error)override{error="Foxglove live support is unavailable";return false;}
};}
std::unique_ptr<LiveSink> createLiveSink(){return std::make_unique<UnavailableLiveSink>();}
}
#endif
