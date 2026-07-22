#include "live_sink.hpp"
#include <foxglove/channel.hpp>
#include <foxglove/error.hpp>
#include <foxglove/server.hpp>
#include <map>
#include <optional>

namespace camera_bridge_mcap { namespace {
class FoxgloveLiveSink final:public LiveSink { public:
 bool start(const std::string&host,uint16_t port,std::string&error)override{
  context_=foxglove::Context::create();foxglove::WebSocketServerOptions o;o.context=context_;o.name="camera_bridge_mcap";o.host=host;o.port=port;o.supported_encodings={"cdr"};auto result=foxglove::WebSocketServer::create(std::move(o));if(!result.has_value()){error=foxglove::strerror(result.error());return false;}server_=std::move(result).value();return true;}
 void stop()override{channels_.clear();if(server_){server_->stop();server_.reset();}}
 bool publish(const std::string&topic,const std::string&type,const std::string&schemaText,const std::vector<uint8_t>&data,uint64_t time_ns,std::string&error)override{
  auto it=channels_.find(topic);if(it==channels_.end()){foxglove::Schema schema;schema.name=type;schema.encoding="ros2msg";schema.data=reinterpret_cast<const std::byte*>(schemaText.data());schema.data_len=schemaText.size();auto result=foxglove::RawChannel::create(topic,"cdr",schema,context_);if(!result.has_value()){error=foxglove::strerror(result.error());return false;}it=channels_.emplace(topic,std::move(result).value()).first;}
  const auto status=it->second.log(reinterpret_cast<const std::byte*>(data.data()),data.size(),time_ns);if(status!=foxglove::FoxgloveError::Ok){error=foxglove::strerror(status);return false;}return true;}
 foxglove::Context context_;std::optional<foxglove::WebSocketServer>server_;std::map<std::string,foxglove::RawChannel>channels_;
};}
std::unique_ptr<LiveSink> createLiveSink(){return std::make_unique<FoxgloveLiveSink>();}
}
