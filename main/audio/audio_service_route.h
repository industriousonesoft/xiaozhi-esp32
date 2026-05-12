#ifndef AUDIO_SERVICE_ROUTE_H
#define AUDIO_SERVICE_ROUTE_H

#include <cstdint>
#include <limits>
#include <vector>

enum class AudioProcessorOutputRoute {
    kEncodeToSendQueue,
    kPlaybackQueue,
};

struct PcmGainResult {
    int peak_before = 0;
    int peak_after = 0;
    uint32_t clipped_samples = 0;
};

inline AudioProcessorOutputRoute SelectAudioProcessorOutputRoute(bool runtime_local_playback,
                                                                 bool compile_time_local_playback) {
    return (runtime_local_playback || compile_time_local_playback)
        ? AudioProcessorOutputRoute::kPlaybackQueue
        : AudioProcessorOutputRoute::kEncodeToSendQueue;
}

inline int PcmAbsPeakValue(int16_t sample) {
    int32_t value = sample;
    if (value < 0) {
        value = -value;
    }
    return value;
}

inline PcmGainResult ApplyPcmGainForLocalPlayback(std::vector<int16_t>& data, float gain) {
    PcmGainResult result;
    if (gain < 0.0f) {
        gain = 0.0f;
    }

    for (auto& sample : data) {
        int before = PcmAbsPeakValue(sample);
        if (before > result.peak_before) {
            result.peak_before = before;
        }

        float scaled = static_cast<float>(sample) * gain;
        int32_t value = static_cast<int32_t>(scaled);
        if (value > std::numeric_limits<int16_t>::max()) {
            value = std::numeric_limits<int16_t>::max();
            result.clipped_samples++;
        } else if (value < std::numeric_limits<int16_t>::min()) {
            value = std::numeric_limits<int16_t>::min();
            result.clipped_samples++;
        }

        sample = static_cast<int16_t>(value);
        int after = PcmAbsPeakValue(sample);
        if (after > result.peak_after) {
            result.peak_after = after;
        }
    }

    return result;
}

#endif // AUDIO_SERVICE_ROUTE_H
