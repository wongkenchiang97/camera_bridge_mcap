#include "camera_bridge_mcap/cdr.hpp"

#include <limits>

namespace camera_bridge_mcap {
CdrWriter::CdrWriter() : data_{0x00, 0x01, 0x00, 0x00} {}
void CdrWriter::align(size_t a) { const size_t n=data_.size()-4; data_.insert(data_.end(),(a-n%a)%a,0); }
void CdrWriter::string(const std::string& v) { if(v.size()>=UINT32_MAX) throw std::length_error("CDR string too large"); primitive<uint32_t>(static_cast<uint32_t>(v.size()+1)); data_.insert(data_.end(),v.begin(),v.end()); data_.push_back(0); }
void CdrWriter::bytes(const uint8_t* p,size_t n) { if(n>UINT32_MAX) throw std::length_error("CDR sequence too large"); primitive<uint32_t>(static_cast<uint32_t>(n)); data_.insert(data_.end(),p,p+n); }
void CdrWriter::fixedDoubles(const double* p,size_t n) { for(size_t i=0;i<n;++i) primitive<double>(p[i]); }
CdrReader::CdrReader(const uint8_t* p,size_t n):data_(p),size_(n) { require(4); if(p[0]!=0||p[1]!=1) throw std::runtime_error("only CDR little endian is supported"); offset_=4; }
void CdrReader::align(size_t a) { const size_t n=offset_-4; offset_+=(a-n%a)%a; require(0); }
void CdrReader::require(size_t n) const { if(offset_>size_||n>size_-offset_) throw std::runtime_error("truncated CDR payload"); }
std::string CdrReader::string() { const auto n=primitive<uint32_t>(); if(!n) throw std::runtime_error("invalid CDR string"); require(n); if(data_[offset_+n-1]) throw std::runtime_error("unterminated CDR string"); std::string v(reinterpret_cast<const char*>(data_+offset_),n-1); offset_+=n; return v; }
std::vector<uint8_t> CdrReader::bytes() { const auto n=primitive<uint32_t>(); require(n); std::vector<uint8_t> v(data_+offset_,data_+offset_+n); offset_+=n; return v; }
void CdrReader::fixedDoubles(double* p,size_t n) { for(size_t i=0;i<n;++i) p[i]=primitive<double>(); }
}  // namespace camera_bridge_mcap
