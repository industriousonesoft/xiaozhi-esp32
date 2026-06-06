#include "audio_debug_service.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <sys/socket.h>

#include <cJSON.h>
#include <esp_app_desc.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>

#define TAG "AudioDebugService"

namespace {
constexpr uint64_t kHeartbeatTimeoutUs = 3'000'000;
constexpr size_t kPacketQueueLength = 32;

bool ReadNumber(const cJSON* object, const char* name, double& value) {
    const cJSON* item = cJSON_GetObjectItemCaseSensitive(object, name);
    if (!cJSON_IsNumber(item)) {
        return false;
    }
    value = item->valuedouble;
    return true;
}

void AddTuningJson(cJSON* parent, const AudioDebugTuningConfig& config) {
    cJSON* tuning = cJSON_AddObjectToObject(parent, "params");
    cJSON_AddNumberToObject(tuning, "pickup_half_angle_deg", config.pickup_half_angle_deg);
    cJSON_AddNumberToObject(tuning, "min_rms_for_voice", config.min_rms_for_voice);
    cJSON_AddNumberToObject(tuning, "min_confidence", config.min_confidence);
    cJSON_AddNumberToObject(tuning, "full_scale_rms", config.full_scale_rms);
    cJSON_AddNumberToObject(tuning, "min_pair_balance", config.min_pair_balance);
    cJSON_AddNumberToObject(tuning, "min_correlation", config.min_correlation);
    cJSON_AddNumberToObject(tuning, "off_axis_min_gain", config.off_axis_min_gain);
    cJSON_AddNumberToObject(tuning, "off_axis_max_gain", config.off_axis_max_gain);
}
} // namespace

AudioDebugService::~AudioDebugService() {
    EndSession();
    if (socket_ >= 0) {
        close(socket_);
    }
    if (packet_queue_ != nullptr) {
        QueuedPacket* packet = nullptr;
        while (xQueueReceive(packet_queue_, &packet, 0) == pdTRUE) {
            heap_caps_free(packet);
        }
        vQueueDelete(packet_queue_);
    }
}

void AudioDebugService::SetCallbacks(AudioDebugServiceCallbacks callbacks) {
    callbacks_ = std::move(callbacks);
}

bool AudioDebugService::Start() {
    if (task_ != nullptr) {
        return true;
    }
    packet_queue_ = xQueueCreate(kPacketQueueLength, sizeof(QueuedPacket*));
    if (packet_queue_ == nullptr) {
        return false;
    }
    BaseType_t result = xTaskCreate(
        TaskEntry, "audio_debug", 6144, this, 2, &task_);
    if (result != pdPASS) {
        vQueueDelete(packet_queue_);
        packet_queue_ = nullptr;
        return false;
    }
    return true;
}

void AudioDebugService::TaskEntry(void* arg) {
    static_cast<AudioDebugService*>(arg)->Run();
    vTaskDelete(nullptr);
}

void AudioDebugService::Run() {
    socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (socket_ < 0) {
        ESP_LOGE(TAG, "socket failed: %d", errno);
        return;
    }
    timeval timeout = {.tv_sec = 0, .tv_usec = 10000};
    setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    sockaddr_in local = {};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(CONFIG_AUDIO_DEBUG_DASHBOARD_CONTROL_PORT);
    if (bind(socket_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
        ESP_LOGE(TAG, "bind port %d failed: %d", CONFIG_AUDIO_DEBUG_DASHBOARD_CONTROL_PORT, errno);
        close(socket_);
        socket_ = -1;
        return;
    }
    ESP_LOGI(TAG, "listening on UDP port %d", CONFIG_AUDIO_DEBUG_DASHBOARD_CONTROL_PORT);

    char buffer[1024];
    while (true) {
        sockaddr_in source = {};
        socklen_t source_size = sizeof(source);
        int received = recvfrom(socket_, buffer, sizeof(buffer), 0,
                                reinterpret_cast<sockaddr*>(&source), &source_size);
        if (received > 0) {
            HandleControl(buffer, received, source);
        }
        DrainPackets();
        if (active_.load() &&
            static_cast<uint64_t>(esp_timer_get_time()) - last_heartbeat_us_ > kHeartbeatTimeoutUs) {
            ESP_LOGW(TAG, "session heartbeat timeout");
            EndSession();
        }
    }
}

void AudioDebugService::HandleControl(
    const char* data, size_t size, const sockaddr_in& source) {
    cJSON* root = cJSON_ParseWithLength(data, size);
    if (root == nullptr) {
        SendResponse(source, "error", "error", "invalid_json");
        return;
    }
    const cJSON* type_item = cJSON_GetObjectItemCaseSensitive(root, "type");
    const char* type = cJSON_IsString(type_item) ? type_item->valuestring : "";
    double session_value = 0;
    const bool has_session = ReadNumber(root, "session_id", session_value);
    const uint32_t requested_session = static_cast<uint32_t>(session_value);

    if (std::strcmp(type, "start") == 0) {
        double version = 0;
        double data_port = 0;
        if (!has_session || !ReadNumber(root, "version", version) ||
            !ReadNumber(root, "data_port", data_port) ||
            version != kAudioDebugProtocolVersion ||
            data_port < 1024 || data_port > 65535) {
            SendResponse(source, type, "error", "invalid_start");
        } else {
            const bool was_active = active_.load();
            QueuedPacket* queued = nullptr;
            while (xQueueReceive(packet_queue_, &queued, 0) == pdTRUE) {
                heap_caps_free(queued);
            }
            if (callbacks_.reset_tuning) {
                callbacks_.reset_tuning();
            }
            {
                std::lock_guard<std::mutex> lock(session_mutex_);
                session_id_ = requested_session;
                destination_ = source;
                destination_.sin_port = htons(static_cast<uint16_t>(data_port));
                packet_builder_ = std::make_unique<AudioDebugPacketBuilder>(session_id_);
                dropped_packets_ = 0;
                last_heartbeat_us_ = esp_timer_get_time();
                active_.store(true);
            }
            if (!was_active && callbacks_.on_session_state) {
                callbacks_.on_session_state(true);
            }
            SendResponse(source, type, "ok");
        }
    } else if (std::strcmp(type, "heartbeat") == 0) {
        if (active_.load() && has_session && requested_session == session_id_) {
            last_heartbeat_us_ = esp_timer_get_time();
            SendResponse(source, type, "ok");
        } else {
            SendResponse(source, type, "error", "invalid_session");
        }
    } else if (std::strcmp(type, "stop") == 0) {
        if (active_.load() && has_session && requested_session == session_id_) {
            SendResponse(source, type, "ok");
            EndSession();
        } else {
            SendResponse(source, type, "error", "invalid_session");
        }
    } else if (std::strcmp(type, "set_params") == 0) {
        const cJSON* params = cJSON_GetObjectItemCaseSensitive(root, "params");
        AudioDebugTuningConfig config;
        double values[8] = {};
        const char* names[8] = {
            "pickup_half_angle_deg", "min_rms_for_voice", "min_confidence",
            "full_scale_rms", "min_pair_balance", "min_correlation",
            "off_axis_min_gain", "off_axis_max_gain",
        };
        bool valid = active_.load() && has_session && requested_session == session_id_ &&
            cJSON_IsObject(params);
        for (int i = 0; valid && i < 8; ++i) {
            valid = ReadNumber(params, names[i], values[i]);
        }
        if (valid) {
            config.pickup_half_angle_deg = values[0];
            config.min_rms_for_voice = values[1];
            config.min_confidence = values[2];
            config.full_scale_rms = values[3];
            config.min_pair_balance = values[4];
            config.min_correlation = values[5];
            config.off_axis_min_gain = values[6];
            config.off_axis_max_gain = values[7];
            valid = callbacks_.set_tuning && callbacks_.set_tuning(config);
        }
        SendResponse(source, type, valid ? "ok" : "error",
                     valid ? nullptr : "invalid_params");
    } else if (std::strcmp(type, "reset_params") == 0) {
        if (active_.load() && has_session && requested_session == session_id_ &&
            callbacks_.reset_tuning) {
            callbacks_.reset_tuning();
            SendResponse(source, type, "ok");
        } else {
            SendResponse(source, type, "error", "invalid_session");
        }
    } else if (std::strcmp(type, "status") == 0) {
        SendResponse(source, type, "ok");
    } else {
        SendResponse(source, "error", "error", "unknown_command");
    }
    cJSON_Delete(root);
}

void AudioDebugService::SendResponse(
    const sockaddr_in& destination, const char* type, const char* status, const char* error) {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", type);
    cJSON_AddStringToObject(root, "status", status);
    cJSON_AddNumberToObject(root, "version", kAudioDebugProtocolVersion);
    cJSON_AddNumberToObject(root, "session_id", session_id_);
    cJSON_AddBoolToObject(root, "active", active_.load());
    cJSON_AddStringToObject(root, "firmware_version", esp_app_get_description()->version);
    cJSON_AddNumberToObject(root, "dropped_packets", dropped_packets_.load());
    cJSON* streams = cJSON_AddObjectToObject(root, "streams");
    cJSON_AddNumberToObject(streams, "raw_mics_sample_rate", 48000);
    cJSON_AddNumberToObject(streams, "raw_mics_channels", 3);
    cJSON_AddNumberToObject(streams, "beam_sample_rate", 48000);
    cJSON_AddNumberToObject(streams, "beam_channels", 1);
    cJSON_AddNumberToObject(streams, "afe_sample_rate", 16000);
    cJSON_AddNumberToObject(streams, "afe_channels", 1);
    if (error != nullptr) {
        cJSON_AddStringToObject(root, "error", error);
    }
    if (callbacks_.get_tuning) {
        AddTuningJson(root, callbacks_.get_tuning());
    }
    char* json = cJSON_PrintUnformatted(root);
    if (json != nullptr) {
        sendto(socket_, json, std::strlen(json), 0,
               reinterpret_cast<const sockaddr*>(&destination), sizeof(destination));
        cJSON_free(json);
    }
    cJSON_Delete(root);
}

void AudioDebugService::EndSession() {
    if (!active_.exchange(false)) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        packet_builder_.reset();
        session_id_ = 0;
    }
    QueuedPacket* packet = nullptr;
    while (packet_queue_ != nullptr && xQueueReceive(packet_queue_, &packet, 0) == pdTRUE) {
        heap_caps_free(packet);
    }
    if (callbacks_.reset_tuning) {
        callbacks_.reset_tuning();
    }
    if (callbacks_.on_session_state) {
        callbacks_.on_session_state(false);
    }
}

void AudioDebugService::QueueRawMics(
    const int16_t* raw, size_t frames, int raw_channels, int sample_rate, uint64_t timestamp_us) {
    if (!active_.load() || raw == nullptr || raw_channels < 4) {
        return;
    }
    std::vector<int16_t> mics(frames * 3);
    for (size_t frame = 0; frame < frames; ++frame) {
        mics[frame * 3] = raw[frame * raw_channels + 1];
        mics[frame * 3 + 1] = raw[frame * raw_channels + 2];
        mics[frame * 3 + 2] = raw[frame * raw_channels + 3];
    }
    QueuePcm(AudioDebugStream::kRawMics, mics.data(), frames, 3, sample_rate, timestamp_us);
}

void AudioDebugService::QueueBeam(
    const int16_t* data, size_t frames, int sample_rate,
    float angle_deg, float confidence, bool active, uint64_t timestamp_us) {
    QueuePcm(AudioDebugStream::kBeam, data, frames, 1, sample_rate, timestamp_us);
    if (!active_.load()) {
        return;
    }
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (packet_builder_) {
        QueuePacket(packet_builder_->BuildDirectionPacket(
            angle_deg, confidence, active, timestamp_us));
    }
}

void AudioDebugService::QueueAfe(
    const int16_t* data, size_t frames, int sample_rate, uint64_t timestamp_us) {
    QueuePcm(AudioDebugStream::kAfe, data, frames, 1, sample_rate, timestamp_us);
}

void AudioDebugService::QueuePcm(
    AudioDebugStream stream, const int16_t* data, size_t frames,
    uint8_t channels, uint32_t sample_rate, uint64_t timestamp_us) {
    if (!active_.load() || data == nullptr || frames == 0) {
        return;
    }
    std::lock_guard<std::mutex> lock(session_mutex_);
    if (!packet_builder_) {
        return;
    }
    auto packets = packet_builder_->BuildPcmPackets(
        stream, data, frames, channels, sample_rate, timestamp_us);
    for (auto& packet : packets) {
        QueuePacket(std::move(packet));
    }
}

void AudioDebugService::QueuePacket(std::vector<uint8_t>&& packet_data) {
    if (packet_data.empty() || packet_data.size() > kAudioDebugMaxDatagramSize) {
        return;
    }
    auto* packet = static_cast<QueuedPacket*>(
        heap_caps_malloc(sizeof(QueuedPacket), MALLOC_CAP_8BIT));
    if (packet == nullptr) {
        ++dropped_packets_;
        return;
    }
    packet->size = packet_data.size();
    std::memcpy(packet->data, packet_data.data(), packet->size);
    if (xQueueSend(packet_queue_, &packet, 0) != pdTRUE) {
        QueuedPacket* oldest = nullptr;
        if (xQueueReceive(packet_queue_, &oldest, 0) == pdTRUE) {
            heap_caps_free(oldest);
        }
        if (xQueueSend(packet_queue_, &packet, 0) == pdTRUE) {
            ++dropped_packets_;
            return;
        }
        heap_caps_free(packet);
        ++dropped_packets_;
    }
}

void AudioDebugService::DrainPackets() {
    if (!active_.load() || packet_queue_ == nullptr) {
        return;
    }
    QueuedPacket* packet = nullptr;
    while (xQueueReceive(packet_queue_, &packet, 0) == pdTRUE) {
        sockaddr_in destination;
        {
            std::lock_guard<std::mutex> lock(session_mutex_);
            destination = destination_;
        }
        if (sendto(socket_, packet->data, packet->size, 0,
                   reinterpret_cast<const sockaddr*>(&destination), sizeof(destination)) < 0) {
            ++dropped_packets_;
        }
        heap_caps_free(packet);
    }
}
