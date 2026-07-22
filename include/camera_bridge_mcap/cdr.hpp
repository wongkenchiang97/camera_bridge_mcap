#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace camera_bridge_mcap {

class CdrWriter {
 public:
  CdrWriter();

  template <typename T>
  void primitive(T value) {
    static_assert(std::is_arithmetic_v<T>, "CDR primitives must be arithmetic");
    align(alignof(T));
    const auto* bytes = reinterpret_cast<const uint8_t*>(&value);
    data_.insert(data_.end(), bytes, bytes + sizeof(T));
  }

  void boolean(bool value) { primitive<uint8_t>(value ? 1 : 0); }
  void string(const std::string& value);
  void bytes(const uint8_t* data, size_t size);
  void fixedDoubles(const double* values, size_t count);
  [[nodiscard]] const std::vector<uint8_t>& data() const { return data_; }
  [[nodiscard]] std::vector<uint8_t> take() { return std::move(data_); }

 private:
  void align(size_t alignment);
  std::vector<uint8_t> data_;
};

class CdrReader {
 public:
  CdrReader(const uint8_t* data, size_t size);

  template <typename T>
  T primitive() {
    static_assert(std::is_arithmetic_v<T>, "CDR primitives must be arithmetic");
    align(alignof(T));
    require(sizeof(T));
    T value{};
    std::memcpy(&value, data_ + offset_, sizeof(T));
    offset_ += sizeof(T);
    return value;
  }

  bool boolean() { return primitive<uint8_t>() != 0; }
  std::string string();
  std::vector<uint8_t> bytes();
  void fixedDoubles(double* values, size_t count);
  [[nodiscard]] size_t remaining() const { return size_ - offset_; }

 private:
  void align(size_t alignment);
  void require(size_t size) const;
  const uint8_t* data_ = nullptr;
  size_t size_ = 0;
  size_t offset_ = 0;
};

}  // namespace camera_bridge_mcap
