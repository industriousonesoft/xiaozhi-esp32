#ifndef AUDIO_DEBUG_SERVICE_H
#define AUDIO_DEBUG_SERVICE_H

#include "audio_debug_protocol.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <netinet/in.h>

struct AudioDebugTuningConfig {
    float pickup_half_angle_deg = 30.0f;
    float min_rms_for_voice = 280.0f;
    float min_confidence = 0.18f;
    float full_scale_rms = 3500.0f;
    float min_pair_balance = 0.75f;
    float min_correlation = 0.35f;
    float off_axis_min_gain = 0.15f;
    float off_axis_max_gain = 0.65f;
};

struct AudioDebugServiceCallbacks {
    std::function<void(bool active)> on_session_state;
    std::function<bool(const AudioDebugTuningConfig& config)> set_tuning;
    std::function<AudioDebugTuningConfig()> get_tuning;
    std::function<void()> reset_tuning;
};

class AudioDebugService {
public:
    AudioDebugService() = default;
    ~AudioDebugService();

    void SetCallbacks(AudioDebugServiceCallbacks callbacks);
    bool Start();
    bool IsActive() const { return active_.load(); }

    void QueueRawMics(const int16_t* raw, size_t frames, int raw_channels,
                      int sample_rate, uint64_t timestamp_us);
    void QueueBeam(const int16_t* data, size_t frames, int sample_rate,
                   float angle_deg, float confidence, bool active, uint64_t timestamp_us);
    void QueueAfe(const int16_t* data, size_t frames, int sample_rate, uint64_t timestamp_us);

private:
    struct QueuedPacket {
        size_t size = 0;
        uint8_t data[kAudioDebugMaxDatagramSize];
    };

    AudioDebugServiceCallbacks callbacks_;
    std::atomic<bool> active_{false};
    std::mutex session_mutex_;
    std::unique_ptr<AudioDebugPacketBuilder> packet_builder_;
    uint32_t session_id_ = 0;
    uint64_t last_heartbeat_us_ = 0;
    std::atomic<uint32_t> dropped_packets_{0};
    int socket_ = -1;
    QueueHandle_t packet_queue_ = nullptr;
    TaskHandle_t task_ = nullptr;
    struct sockaddr_in destination_ = {};

    static void TaskEntry(void* arg);
    void Run();
    void HandleControl(const char* data, size_t size, const struct sockaddr_in& source);
    void SendResponse(const struct sockaddr_in& destination, const char* type,
                      const char* status, const char* error = nullptr);
    void EndSession();
    void QueuePcm(AudioDebugStream stream, const int16_t* data, size_t frames,
                  uint8_t channels, uint32_t sample_rate, uint64_t timestamp_us);
    void QueuePacket(std::vector<uint8_t>&& packet);
    void DrainPackets();
};

#endif
