#include "camera_bridge_mcap/producer.hpp"
#include "camera_bridge_mcap/ros2_messages.hpp"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <mcap/reader.hpp>
#include <mutex>
#include <map>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace camera_bridge_mcap {
namespace {
    constexpr uint8_t kColorStream = 1;
    constexpr uint8_t kDepthStream = 2;
    constexpr uint8_t kImuStream = 3;
    constexpr uint8_t kRightStream = 4;
    constexpr uint8_t kAccelStream = 5;
    constexpr uint8_t kInfrared1Stream = 6;
    constexpr uint8_t kInfrared2Stream = 7;
    constexpr uint8_t kDerivedImuStream = 8;

    struct TimingKey {
        uint32_t source_id;
        uint8_t stream_kind;
        uint64_t host_timestamp_us;
        bool operator==(const TimingKey& other) const { return source_id == other.source_id && stream_kind == other.stream_kind && host_timestamp_us == other.host_timestamp_us; }
    };
    struct TimingKeyHash {
        size_t operator()(const TimingKey& key) const
        {
            size_t value = std::hash<uint64_t> {}(key.host_timestamp_us);
            value ^= std::hash<uint32_t> {}(key.source_id) + 0x9e3779b9 + (value << 6) + (value >> 2);
            value ^= std::hash<uint8_t> {}(key.stream_kind) + 0x9e3779b9 + (value << 6) + (value >> 2);
            return value;
        }
    };
    using TimingIndex = std::unordered_map<TimingKey, TimingMessage, TimingKeyHash>;
    using AmbiguousTiming = std::unordered_set<TimingKey, TimingKeyHash>;

    class TimedReadable final : public mcap::IReadable {
    public:
        explicit TimedReadable(mcap::IReadable& source)
            : source_(source) {}

        uint64_t size() const override { return source_.size(); }

        uint64_t read(std::byte** output, uint64_t offset, uint64_t size) override
        {
            const auto begin = std::chrono::steady_clock::now();
            const uint64_t bytes = source_.read(output, offset, size);
            const auto duration = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - begin).count());
            ++interval_samples_;
            interval_total_us_ += duration;
            interval_max_us_ = std::max(interval_max_us_, duration);
            interval_bytes_ += bytes;
            return bytes;
        }

        void consumeInterval(uint64_t* samples, uint64_t* total_us,
            uint64_t* max_us, uint64_t* bytes)
        {
            *samples = std::exchange(interval_samples_, 0);
            *total_us = std::exchange(interval_total_us_, 0);
            *max_us = std::exchange(interval_max_us_, 0);
            *bytes = std::exchange(interval_bytes_, 0);
        }

    private:
        mcap::IReadable& source_;
        uint64_t interval_samples_ = 0;
        uint64_t interval_total_us_ = 0;
        uint64_t interval_max_us_ = 0;
        uint64_t interval_bytes_ = 0;
    };

    bridge::FrameTiming frameTiming(const TimingMessage& t)
    {
        bridge::FrameTiming v;
        v.driver_receive_steady_us = t.driver_receive_steady_us;
        v.capture_steady_us = t.capture_steady_us;
        v.capture_steady_valid = t.capture_steady_valid;
        v.timestamp_source = static_cast<bridge::CaptureTimestampSource>(t.timestamp_source);
        v.clock_mapping_uncertainty_us = t.clock_mapping_uncertainty_us;
        return v;
    }
    bridge::CameraDistortionModel distortion(const std::string& s) { return s == "equidistant" ? bridge::CameraDistortionModel::KannalaBrandt4 : s.empty() ? bridge::CameraDistortionModel::None
                                                                                                                                                           : bridge::CameraDistortionModel::BrownConrady; }
    uint32_t sourceFromTopic(const std::string& topic)
    {
        constexpr char prefix[] = "/camera";
        if (topic.rfind(prefix, 0) != 0)
            throw std::runtime_error("invalid camera topic");
        size_t end = sizeof(prefix) - 1;
        while (end < topic.size() && topic[end] >= '0' && topic[end] <= '9')
            ++end;
        if (end == sizeof(prefix) - 1 || end >= topic.size() || topic[end] != '/')
            throw std::runtime_error("invalid camera source topic");
        return static_cast<uint32_t>(std::stoul(topic.substr(sizeof(prefix) - 1, end - (sizeof(prefix) - 1))));
    }
    uint32_t sourceFromFrameId(const std::string& frame_id)
    {
        constexpr char prefix[] = "camera";
        if (frame_id.rfind(prefix, 0) != 0)
            return 0; // Legacy single-camera transforms used generic frame IDs.
        size_t end = sizeof(prefix) - 1;
        while (end < frame_id.size() && frame_id[end] >= '0' && frame_id[end] <= '9')
            ++end;
        if (end == sizeof(prefix) - 1 || end >= frame_id.size() || frame_id[end] != '_')
            return 0;
        return static_cast<uint32_t>(std::stoul(frame_id.substr(sizeof(prefix) - 1, end - (sizeof(prefix) - 1))));
    }
    std::optional<TimingMessage> findTiming(const TimingIndex& index,
        const AmbiguousTiming& ambiguous, uint32_t source, uint8_t kind, uint64_t host)
    {
        const TimingKey key { source, kind, host };
        if (ambiguous.count(key)) return std::nullopt;
        const auto found = index.find(key);
        if (found == index.end())
            return std::nullopt;
        return found->second;
    }
}
class Ros2McapProducer::Impl {
public:
    explicit Impl(Options o)
        : options(std::move(o))
    {
    }
    bool start(std::string* error)
    {
        if (active.exchange(true))
            return true;
        if (worker.joinable())
            worker.join();
        mcap::McapReader probe;
        auto s = probe.open(options.input_path.string());
        if (!s.ok()) {
            active = false;
            if (error)
                *error = s.message;
            return false;
        }
        probe.close();
        worker = std::thread([this] { run(); });
        return true;
    }
    void stop()
    {
        active = false;
        if (worker.joinable())
            worker.join();
    }
    void run()
    {
        mcap::McapReader reader;
        auto s = reader.open(options.input_path.string());
        if (!s.ok()) {
            active = false;
            return;
        }
        TimingIndex timingIndex;
        AmbiguousTiming ambiguousTiming;
        for (const auto& v : reader.readMessages()) {
            if (!active)
                break;
            const auto& timingTopic = v.channel->topic;
            if (timingTopic != "/amr/camera_timing" &&
                timingTopic.find("/imu/gyro/device_time") == std::string::npos &&
                timingTopic.find("/imu/accel/device_time") == std::string::npos &&
                timingTopic.find("/imu/data/device_time") == std::string::npos)
                continue;
            try {
                auto timing = decodeTiming(reinterpret_cast<const uint8_t*>(v.message.data), static_cast<size_t>(v.message.dataSize));
                const TimingKey key { timing.source_id, timing.stream_kind, timing.host_timestamp_us };
                if (!ambiguousTiming.count(key) &&
                    !timingIndex.emplace(key, timing).second) {
                    timingIndex.erase(key);
                    ambiguousTiming.insert(key);
                }
            } catch (const std::exception&) {
                active = false;
                reader.close();
                return;
            }
        }
        reader.close();
        if (!active)
            return;
        std::FILE* replay_file = std::fopen(options.input_path.string().c_str(), "rb");
        if (!replay_file) {
            active = false;
            return;
        }
        mcap::FileReader replay_file_reader(replay_file);
        TimedReadable replay_timed_reader(replay_file_reader);
        s = reader.open(replay_timed_reader);
        if (!s.ok()) {
            std::fclose(replay_file);
            active = false;
            return;
        }
        uint64_t first = 0;
        auto wall = std::chrono::steady_clock::now();
        auto previous_iteration_end = std::chrono::steady_clock::now();
        const auto elapsedUs = [](const auto begin, const auto end) {
            return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count());
        };
        const auto recordStage = [this](uint64_t duration_us,
                                     uint64_t bridge::ProducerStats::*samples,
                                     uint64_t bridge::ProducerStats::*total_us,
                                     uint64_t bridge::ProducerStats::*max_us) {
            std::lock_guard<std::mutex> lock(mu);
            ++(stats.*samples);
            stats.*total_us += duration_us;
            stats.*max_us = std::max(stats.*max_us, duration_us);
        };
        for (const auto& v : reader.readMessages()) {
            const auto iteration_begin = std::chrono::steady_clock::now();
            const uint64_t iterator_advance_us =
                elapsedUs(previous_iteration_end, iteration_begin);
            recordStage(iterator_advance_us,
                &bridge::ProducerStats::iterator_advance_samples,
                &bridge::ProducerStats::iterator_advance_total_us,
                &bridge::ProducerStats::iterator_advance_max_us);
            uint64_t source_read_samples = 0;
            uint64_t source_read_us = 0;
            uint64_t source_read_max_us = 0;
            uint64_t source_read_bytes = 0;
            replay_timed_reader.consumeInterval(
                &source_read_samples, &source_read_us, &source_read_max_us,
                &source_read_bytes);
            {
                std::lock_guard<std::mutex> lock(mu);
                stats.source_read_samples += source_read_samples;
                stats.source_read_total_us += source_read_us;
                stats.source_read_max_us =
                    std::max(stats.source_read_max_us, source_read_max_us);
                stats.source_read_bytes += source_read_bytes;
            }
            recordStage(
                iterator_advance_us > source_read_us
                    ? iterator_advance_us - source_read_us
                    : 0,
                &bridge::ProducerStats::iterator_internal_samples,
                &bridge::ProducerStats::iterator_internal_total_us,
                &bridge::ProducerStats::iterator_internal_max_us);
            if (!active)
                break;
            const auto topic = v.channel->topic;
            const auto* p = reinterpret_cast<const uint8_t*>(v.message.data);
            const auto n = static_cast<size_t>(v.message.dataSize);
            try {
                if (topic == "/amr/camera_timing" ||
                    topic.find("/imu/gyro/device_time") != std::string::npos ||
                    topic.find("/imu/accel/device_time") != std::string::npos ||
                    topic.find("/imu/data/device_time") != std::string::npos) {
                    previous_iteration_end = std::chrono::steady_clock::now();
                    continue;
                }
                uint64_t us = v.message.publishTime / 1000;
                if (!first) {
                    first = us;
                    wall = std::chrono::steady_clock::now();
                }
                const auto replay_wait_begin = std::chrono::steady_clock::now();
                if (options.replay_speed > 0 && us >= first)
                    std::this_thread::sleep_until(wall + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>((us - first) * 1e-6 / options.replay_speed)));
                recordStage(elapsedUs(replay_wait_begin, std::chrono::steady_clock::now()),
                    &bridge::ProducerStats::replay_wait_samples,
                    &bridge::ProducerStats::replay_wait_total_us,
                    &bridge::ProducerStats::replay_wait_max_us);
                bridge::IFrameConsumer* c = nullptr;
                {
                    std::lock_guard<std::mutex> l(mu);
                    c = consumer;
                }
                if (!c) {
                    previous_iteration_end = std::chrono::steady_clock::now();
                    continue;
                }
                const auto decode_construct_begin = std::chrono::steady_clock::now();
                if(topic.find("/recorder/parameters") != std::string::npos) {
                    const auto decoded=decodeDiagnosticArray(p,n);
                    if(decoded.status.empty())throw std::runtime_error("empty camera parameter diagnostic");
                    bridge::CameraParametersEvent e;
                    e.source_id=sourceFromTopic(topic);
                    e.timestamp_us=timestampUs(decoded.header.stamp);
                    e.serial_number=decoded.status.front().hardware_id;
                    std::map<std::string,bridge::CameraParameterValue> values;
                    for(const auto& kv:decoded.status.front().values) {
                        if(kv.key=="sdk_version") {e.sdk_version=kv.value;continue;}
                        const auto dot=kv.key.rfind('.');
                        if(dot==std::string::npos)continue;
                        const auto name=kv.key.substr(0,dot);
                        const auto field=kv.key.substr(dot+1);
                        auto& value=values[name];
                        value.name=name;
                        if(field=="requested")value.requested=kv.value;
                        else if(field=="effective")value.effective=kv.value;
                        else if(field=="origin")value.origin=kv.value;
                    }
                    for(auto& [name,value]:values)e.values.push_back(std::move(value));
                    c->onCameraParameters(e);
                } else if (topic.find("/color/image_raw") != std::string::npos || topic.find("/right/image_raw") != std::string::npos || topic.find("/depth/image_raw") != std::string::npos || topic.find("/infrared1/image_raw") != std::string::npos || topic.find("/infrared2/image_raw") != std::string::npos) {
                    auto m = decodeImage(p, n);
                    us = timestampUs(m.header.stamp);
                    const uint32_t source = sourceFromTopic(topic);
                    const bool depth = topic.find("/depth/") != std::string::npos;
                    const bool right = topic.find("/right/") != std::string::npos;
                    const bool infrared1 = topic.find("/infrared1/") != std::string::npos;
                    const bool infrared2 = topic.find("/infrared2/") != std::string::npos;
                    const uint8_t stream = depth ? kDepthStream : right ? kRightStream :
                        infrared1 ? kInfrared1Stream : infrared2 ? kInfrared2Stream : kColorStream;
                    const auto timing = findTiming(timingIndex, ambiguousTiming, source, stream, us);
                    if (infrared1 || infrared2) {
                        if (m.encoding != "mono8" || m.step < m.width ||
                            m.data.size() < static_cast<size_t>(m.step) * m.height)
                            throw std::runtime_error("invalid infrared image");
                        bridge::InfraredFrameEvent e;
                        e.timestamp_us = us;
                        e.device_timestamp_us = timing ? timing->device_timestamp_us : us;
                        e.source_id = source;
                        e.sensor_index = infrared1 ? 1 : 2;
                        e.timing = timing ? frameTiming(*timing) : bridge::FrameTiming {};
                        e.frame_id = m.header.frame_id;
                        e.mono8 = cv::Mat(m.height,m.width,CV_8UC1,m.data.data(),m.step).clone();
                        const auto callback_begin = std::chrono::steady_clock::now();
                        recordStage(elapsedUs(decode_construct_begin, callback_begin),
                            &bridge::ProducerStats::decode_construct_samples,
                            &bridge::ProducerStats::decode_construct_total_us,
                            &bridge::ProducerStats::decode_construct_max_us);
                        c->onInfraredFrame(e);
                        recordStage(elapsedUs(callback_begin, std::chrono::steady_clock::now()),
                            &bridge::ProducerStats::consumer_callback_samples,
                            &bridge::ProducerStats::consumer_callback_total_us,
                            &bridge::ProducerStats::consumer_callback_max_us);
                    } else if (depth) {
                        if (m.encoding != "16UC1" || m.step < m.width * 2 || m.data.size() < static_cast<size_t>(m.step) * m.height)
                            throw std::runtime_error("invalid depth image");
                        bridge::DepthFrameEvent e;
                        e.timestamp_us = us;
                        e.device_timestamp_us = timing ? timing->device_timestamp_us : us;
                        e.source_id = source;
                        e.timing = timing ? frameTiming(*timing) : bridge::FrameTiming {};
                        e.frame_id = m.header.frame_id;
                        e.depth_mono16 = cv::Mat(m.height, m.width, CV_16UC1, m.data.data(), m.step).clone();
                        const auto callback_begin = std::chrono::steady_clock::now();
                        recordStage(elapsedUs(decode_construct_begin, callback_begin),
                            &bridge::ProducerStats::decode_construct_samples,
                            &bridge::ProducerStats::decode_construct_total_us,
                            &bridge::ProducerStats::decode_construct_max_us);
                        c->onDepthFrame(e);
                        recordStage(elapsedUs(callback_begin, std::chrono::steady_clock::now()),
                            &bridge::ProducerStats::consumer_callback_samples,
                            &bridge::ProducerStats::consumer_callback_total_us,
                            &bridge::ProducerStats::consumer_callback_max_us);
                        std::lock_guard<std::mutex> l(mu);
                        ++stats.depth_frames_received;
                        ++stats.depth_frames_decoded;
                    } else if (right) {
                        if (m.encoding != "bgr8" || m.step < m.width * 3 || m.data.size() < static_cast<size_t>(m.step) * m.height)
                            throw std::runtime_error("invalid right image");
                        bridge::RightFrameEvent e;
                        e.timestamp_us = us;
                        e.device_timestamp_us = timing ? timing->device_timestamp_us : us;
                        e.source_id = source;
                        e.timing = timing ? frameTiming(*timing) : bridge::FrameTiming {};
                        e.frame_id = m.header.frame_id;
                        e.bgr = cv::Mat(m.height, m.width, CV_8UC3, m.data.data(), m.step).clone();
                        const auto callback_begin = std::chrono::steady_clock::now();
                        recordStage(elapsedUs(decode_construct_begin, callback_begin),
                            &bridge::ProducerStats::decode_construct_samples,
                            &bridge::ProducerStats::decode_construct_total_us,
                            &bridge::ProducerStats::decode_construct_max_us);
                        c->onRightFrame(e);
                        recordStage(elapsedUs(callback_begin, std::chrono::steady_clock::now()),
                            &bridge::ProducerStats::consumer_callback_samples,
                            &bridge::ProducerStats::consumer_callback_total_us,
                            &bridge::ProducerStats::consumer_callback_max_us);
                        std::lock_guard<std::mutex> l(mu);
                        ++stats.right_frames_received;
                        ++stats.right_frames_decoded;
                    } else {
                        if (m.encoding != "bgr8" || m.step < m.width * 3 || m.data.size() < static_cast<size_t>(m.step) * m.height)
                            throw std::runtime_error("invalid color image");
                        bridge::ColorFrameEvent e;
                        e.timestamp_us = us;
                        e.device_timestamp_us = timing ? timing->device_timestamp_us : us;
                        e.source_id = source;
                        e.timing = timing ? frameTiming(*timing) : bridge::FrameTiming {};
                        e.frame_id = m.header.frame_id;
                        e.bgr = cv::Mat(m.height, m.width, CV_8UC3, m.data.data(), m.step).clone();
                        const auto callback_begin = std::chrono::steady_clock::now();
                        recordStage(elapsedUs(decode_construct_begin, callback_begin),
                            &bridge::ProducerStats::decode_construct_samples,
                            &bridge::ProducerStats::decode_construct_total_us,
                            &bridge::ProducerStats::decode_construct_max_us);
                        c->onColorFrame(e);
                        recordStage(elapsedUs(callback_begin, std::chrono::steady_clock::now()),
                            &bridge::ProducerStats::consumer_callback_samples,
                            &bridge::ProducerStats::consumer_callback_total_us,
                            &bridge::ProducerStats::consumer_callback_max_us);
                        std::lock_guard<std::mutex> l(mu);
                        ++stats.color_frames_received;
                        ++stats.color_frames_decoded;
                    }
                } else if (topic.find("/imu/data_raw") != std::string::npos ||
                           topic.find("/imu/data") != std::string::npos ||
                           topic.find("/imu/gyro") != std::string::npos ||
                           topic.find("/imu/accel") != std::string::npos) {
                    auto m = decodeImu(p, n);
                    bridge::ImuSampleEvent e;
                    e.timestamp_us = timestampUs(m.header.stamp);
                    const uint32_t source = sourceFromTopic(topic);
                    const bool rawGyro = topic.find("/imu/gyro") != std::string::npos;
                    const bool rawAccel = topic.find("/imu/accel") != std::string::npos;
                    const bool derivedImu = topic.find("/imu/data") != std::string::npos &&
                                            topic.find("/imu/data_raw") == std::string::npos;
                    const auto timing = findTiming(timingIndex, ambiguousTiming, source,
                        derivedImu ? kDerivedImuStream : rawAccel ? kAccelStream : kImuStream, e.timestamp_us);
                    if ((rawGyro || rawAccel || derivedImu) && !timing) {
                        previous_iteration_end = std::chrono::steady_clock::now();
                        continue;
                    }
                    e.device_timestamp_us = timing ? timing->device_timestamp_us : e.timestamp_us;
                    e.source_id = source;
                    e.frame_id = m.header.frame_id;
                    e.timing = timing ? frameTiming(*timing) : bridge::FrameTiming {};
                    e.has_gyro = !rawAccel;
                    e.gyro = { static_cast<float>(m.angular_velocity[0]), static_cast<float>(m.angular_velocity[1]), static_cast<float>(m.angular_velocity[2]) };
                    e.has_accel = !rawGyro;
                    e.accel = { static_cast<float>(m.linear_acceleration[0]), static_cast<float>(m.linear_acceleration[1]), static_cast<float>(m.linear_acceleration[2]) };
                    const auto callback_begin = std::chrono::steady_clock::now();
                    recordStage(elapsedUs(decode_construct_begin, callback_begin),
                        &bridge::ProducerStats::decode_construct_samples,
                        &bridge::ProducerStats::decode_construct_total_us,
                        &bridge::ProducerStats::decode_construct_max_us);
                    if(derivedImu)c->onDerivedImuSample(e);
                    else c->onImuSample(e);
                    recordStage(elapsedUs(callback_begin, std::chrono::steady_clock::now()),
                        &bridge::ProducerStats::consumer_callback_samples,
                        &bridge::ProducerStats::consumer_callback_total_us,
                        &bridge::ProducerStats::consumer_callback_max_us);
                    std::lock_guard<std::mutex> l(mu);
                    ++stats.imu_framesets_received;
                    ++stats.imu_accel_samples;
                    ++stats.imu_gyro_samples;
                } else if (topic.find("/color/camera_info") != std::string::npos || topic.find("/right/camera_info") != std::string::npos || topic.find("/depth/camera_info") != std::string::npos || topic.find("/infrared1/camera_info") != std::string::npos || topic.find("/infrared2/camera_info") != std::string::npos) {
                    auto m = decodeCameraInfo(p, n);
                    const uint32_t source = sourceFromTopic(topic);
                    auto& e = calibrationBySource[source];
                    e.source_id = source;
                    e.timestamp_us = std::max(e.timestamp_us, timestampUs(m.header.stamp));
                    auto set = [&](bridge::CameraIntrinsic& i, bridge::CameraDistortion& d) {i.width=m.width;i.height=m.height;i.fx=m.k[0];i.fy=m.k[4];i.cx=m.k[2];i.cy=m.k[5];d.model=distortion(m.distortion_model);if(m.d.size()>0)d.k1=m.d[0];if(m.d.size()>1)d.k2=m.d[1];if(m.d.size()>2)d.p1=m.d[2];if(m.d.size()>3)d.p2=m.d[3];if(m.d.size()>4)d.k3=m.d[4];if(m.d.size()>5)d.k4=m.d[5];if(m.d.size()>6)d.k5=m.d[6];if(m.d.size()>7)d.k6=m.d[7]; };
                    if (topic.find("/color/") != std::string::npos) {
                        e.has_color = true;
                        e.color_frame_id = m.header.frame_id;
                        set(e.color_intrinsic, e.color_distortion);
                    } else if (topic.find("/right/") != std::string::npos) {
                        e.has_right = true;
                        e.right_frame_id = m.header.frame_id;
                        set(e.right_intrinsic, e.right_distortion);
                    } else if (topic.find("/infrared1/") != std::string::npos) {
                        e.has_infrared1 = true;
                        e.infrared1_frame_id = m.header.frame_id;
                        set(e.infrared1_intrinsic, e.infrared1_distortion);
                    } else if (topic.find("/infrared2/") != std::string::npos) {
                        e.has_infrared2 = true;
                        e.infrared2_frame_id = m.header.frame_id;
                        set(e.infrared2_intrinsic, e.infrared2_distortion);
                    } else {
                        e.has_depth = true;
                        e.depth_frame_id = m.header.frame_id;
                        set(e.depth_intrinsic, e.depth_distortion);
                    }
                    const auto callback_begin = std::chrono::steady_clock::now();
                    recordStage(elapsedUs(decode_construct_begin, callback_begin),
                        &bridge::ProducerStats::decode_construct_samples,
                        &bridge::ProducerStats::decode_construct_total_us,
                        &bridge::ProducerStats::decode_construct_max_us);
                    c->onCameraCalibration(e);
                    recordStage(elapsedUs(callback_begin, std::chrono::steady_clock::now()),
                        &bridge::ProducerStats::consumer_callback_samples,
                        &bridge::ProducerStats::consumer_callback_total_us,
                        &bridge::ProducerStats::consumer_callback_max_us);
                } else if (topic == "/tf_static") {
                    auto m = decodeTf(p, n);
                    bridge::ExtrinsicsEvent e;
                    if (!m.transforms.empty()) {
                        e.timestamp_us = timestampUs(m.transforms.front().header.stamp);
                        e.source_id = sourceFromFrameId(m.transforms.front().header.frame_id);
                    }
                    for (const auto& t : m.transforms) {
                        bridge::ExtrinsicTransformEvent x;
                        x.parent_frame_id = t.header.frame_id;
                        x.child_frame_id = t.child_frame_id;
                        x.extrinsic.translation_scale_to_meters = 1.0;
                        for (int i = 0; i < 3; ++i)
                            x.extrinsic.trans[i] = static_cast<float>(t.translation[i]);
                        const double qx = t.rotation[0], qy = t.rotation[1], qz = t.rotation[2], qw = t.rotation[3];
                        auto* r = x.extrinsic.rot;
                        r[0] = 1 - 2 * (qy * qy + qz * qz);
                        r[1] = 2 * (qx * qy - qz * qw);
                        r[2] = 2 * (qx * qz + qy * qw);
                        r[3] = 2 * (qx * qy + qz * qw);
                        r[4] = 1 - 2 * (qx * qx + qz * qz);
                        r[5] = 2 * (qy * qz - qx * qw);
                        r[6] = 2 * (qx * qz - qy * qw);
                        r[7] = 2 * (qy * qz + qx * qw);
                        r[8] = 1 - 2 * (qx * qx + qy * qy);
                        e.transforms.push_back(std::move(x));
                    }
                    const auto callback_begin = std::chrono::steady_clock::now();
                    recordStage(elapsedUs(decode_construct_begin, callback_begin),
                        &bridge::ProducerStats::decode_construct_samples,
                        &bridge::ProducerStats::decode_construct_total_us,
                        &bridge::ProducerStats::decode_construct_max_us);
                    c->onExtrinsics(e);
                    recordStage(elapsedUs(callback_begin, std::chrono::steady_clock::now()),
                        &bridge::ProducerStats::consumer_callback_samples,
                        &bridge::ProducerStats::consumer_callback_total_us,
                        &bridge::ProducerStats::consumer_callback_max_us);
                }
                previous_iteration_end = std::chrono::steady_clock::now();
            } catch (const std::exception&) {
                active = false;
                break;
            }
        }
        reader.close();
        std::fclose(replay_file);
        active = false;
    }
    Options options;
    std::atomic<bool> active { false };
    std::thread worker;
    mutable std::mutex mu;
    bridge::IFrameConsumer* consumer = nullptr;
    bridge::ProducerStats stats;
    std::unordered_map<uint32_t, bridge::CameraCalibrationEvent> calibrationBySource;
};
Ros2McapProducer::Ros2McapProducer(Options o)
    : impl_(std::make_unique<Impl>(std::move(o)))
{
}
Ros2McapProducer::~Ros2McapProducer() { stop(); }
void Ros2McapProducer::setFrameConsumer(bridge::IFrameConsumer* c)
{
    std::lock_guard<std::mutex> l(impl_->mu);
    impl_->consumer = c;
}
void Ros2McapProducer::start()
{
    std::string error;
    if (!impl_->start(&error))
        throw std::runtime_error(error);
}
bool Ros2McapProducer::tryStart(std::string* e) { return impl_->start(e); }
void Ros2McapProducer::stop() { impl_->stop(); }
bool Ros2McapProducer::running() const { return impl_->active; }
bridge::ProducerStats Ros2McapProducer::consumeStats()
{
    std::lock_guard<std::mutex> l(impl_->mu);
    auto s = impl_->stats;
    impl_->stats = {};
    return s;
}
} // namespace camera_bridge_mcap
