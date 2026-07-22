#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace camera_bridge_mcap {
class LiveSink {
 public:
  virtual ~LiveSink() = default;
  virtual bool start(const std::string& host,uint16_t port,std::string& error)=0;
  virtual void stop()=0;
  virtual bool publish(const std::string& topic,const std::string& type,
      const std::string& schema,const std::vector<uint8_t>& data,uint64_t time_ns,
      std::string& error)=0;
};
std::unique_ptr<LiveSink> createLiveSink();
}
