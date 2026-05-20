#include "korvo1_mic_array_processor.h"

#include <algorithm>
#include <cmath>

#ifdef ESP_PLATFORM
#include <esp_log.h>
#endif

namespace {
constexpr const char* TAG = "Korvo1MicArray";

#ifdef ESP_PLATFORM
#define KORVO1_MIC_ARRAY_LOGI(...) ESP_LOGI(TAG, __VA_ARGS__)
#else
#define KORVO1_MIC_ARRAY_LOGI(...) do {} while (0)
#endif

// ES7210 原始 I2S 读出按 4 路 int16 lane 处理。根据 esp-skainet Korvo-1 BSP，lane0 是回声参考；
// Korvo-1 硬件有三颗模拟麦，v2 默认把 lane1/lane2/lane3 都作为麦阵输入。
constexpr int kRawChannels = 4;
constexpr int kRawRefIndex = 0;
constexpr int kRawMic0Index = 1;
constexpr int kRawMic1Index = 2;
constexpr int kRawMic2Index = 3;

// 麦阵输出给后级 AFE 的格式固定为 M/R：一路增强后的主声源 + 一路播放参考。
constexpr int kAfeChannels = 2;
constexpr int kMicArrayChannels = 3;

constexpr int kSampleRate = 16000;
constexpr float kMicDistanceMeters = 0.065f;
constexpr float kSpeedOfSoundMetersPerSec = 343.0f;

// Beamformer 的经验门限。这里判断的是三路麦里最强一路的 10ms/16ms RMS。
// 太低会被底噪触发，太高会漏掉远距离/小音量说话。
constexpr float kMinRmsForVoice = 280.0f;
constexpr float kFullScaleRms = 3500.0f;
constexpr float kMinConfidence = 0.18f;
constexpr uint32_t kDebugLogIntervalMs = 500;
constexpr float kPi = 3.14159265358979323846f;

// Korvo-1 三麦按圆阵近似建模。真实零度和顺逆时针可通过 LED 映射宏校准；
// 若实测通道物理顺序不同，优先调整 raw mic index，而不是改通用算法。
constexpr float kMicAnglesDeg[kMicArrayChannels] = {0.0f, 120.0f, 240.0f};

constexpr float kMainMicWeight = 0.82f;
constexpr float kSideMicWeight = 0.09f;
constexpr float kMainDominanceRatio = 0.85f;

float Clamp(float value, float min_value, float max_value) {
    return std::max(min_value, std::min(value, max_value));
}

float DegToRad(float angle_deg) {
    return angle_deg * kPi / 180.0f;
}

int16_t ClampToInt16(float value) {
    value = Clamp(value, -32768.0f, 32767.0f);
    return static_cast<int16_t>(std::lround(value));
}

float SampleWithDelay(const int16_t* mic_data, size_t frames, int ch, size_t i, float delay_samples) {
    const float source_index = static_cast<float>(i) - delay_samples;
    if (source_index <= 0.0f) {
        return static_cast<float>(mic_data[ch]);
    }
    const float last_index = static_cast<float>(frames - 1);
    if (source_index >= last_index) {
        return static_cast<float>(mic_data[kMicArrayChannels * (frames - 1) + ch]);
    }

    const size_t index0 = static_cast<size_t>(source_index);
    const size_t index1 = index0 + 1;
    const float frac = source_index - static_cast<float>(index0);
    const float sample0 = static_cast<float>(mic_data[kMicArrayChannels * index0 + ch]);
    const float sample1 = static_cast<float>(mic_data[kMicArrayChannels * index1 + ch]);
    return sample0 + (sample1 - sample0) * frac;
}
} // namespace

Korvo1MicArrayProcessor::~Korvo1MicArrayProcessor() {
    Reset();
}

Korvo1MicArrayResult Korvo1MicArrayProcessor::ProcessRaw(const int16_t* raw_data, size_t raw_samples,
                                                         std::vector<int16_t>& output_mr,
                                                         uint32_t timestamp_ms) {
    Korvo1MicArrayResult result;
    result.timestamp_ms = timestamp_ms;
    output_mr.clear();

    if (raw_data == nullptr || raw_samples < kRawChannels || raw_samples % kRawChannels != 0) {
        if (ShouldLogDebug(timestamp_ms)) {
            KORVO1_MIC_ARRAY_LOGI("inactive: invalid raw frame data=%p samples=%u",
                                  raw_data, static_cast<unsigned>(raw_samples));
        }
        return result;
    }

    const size_t frames = raw_samples / kRawChannels;
    std::vector<int16_t> mic_data(frames * kMicArrayChannels);
    std::vector<int16_t> ref_data(frames);
    for (size_t i = 0; i < frames; ++i) {
        ref_data[i] = raw_data[kRawChannels * i + kRawRefIndex];
        mic_data[kMicArrayChannels * i + 0] = raw_data[kRawChannels * i + kRawMic0Index];
        mic_data[kMicArrayChannels * i + 1] = raw_data[kRawChannels * i + kRawMic1Index];
        mic_data[kMicArrayChannels * i + 2] = raw_data[kRawChannels * i + kRawMic2Index];
    }

    std::vector<int16_t> enhanced_mic;
    result = ProcessFixedMainMicBeamformer(mic_data.data(), frames, enhanced_mic, timestamp_ms);

    output_mr.resize(frames * kAfeChannels);
    for (size_t i = 0; i < frames; ++i) {
        output_mr[kAfeChannels * i + 0] = enhanced_mic[i];
        output_mr[kAfeChannels * i + 1] = ref_data[i];
    }

    return result;
}

Korvo1MicArrayResult Korvo1MicArrayProcessor::Process(const int16_t* data, size_t samples, uint32_t timestamp_ms) {
    std::vector<int16_t> output_mr;
    return ProcessRaw(data, samples, output_mr, timestamp_ms);
}

Korvo1MicArrayResult Korvo1MicArrayProcessor::ProcessFixedMainMicBeamformer(const int16_t* mic_data, size_t frames,
                                                                             std::vector<int16_t>& output_mic,
                                                                             uint32_t timestamp_ms) {
    // 固定第 1 颗 MIC 为主拾音方向。这里不做动态 DOA 选择，也不依赖乐鑫 MASE；
    // Mic1 始终是输出主导，Mic2/Mic3 只在与主方向相干时提供少量增强。
    Korvo1MicArrayResult result;
    result.timestamp_ms = timestamp_ms;
    output_mic.resize(frames);

    if (mic_data == nullptr || frames == 0) {
        return result;
    }

    double energy[kMicArrayChannels] = {};
    for (size_t i = 0; i < frames; ++i) {
        for (int ch = 0; ch < kMicArrayChannels; ++ch) {
            // 累加平方能量。平方有两个作用：
            //   1. 把正负波形都变成正能量，避免正负样本互相抵消；
            //   2. 对大幅度声音更敏感，适合做短时响度判断。
            const float sample = static_cast<float>(mic_data[kMicArrayChannels * i + ch]);
            energy[ch] += sample * sample;
        }
    }

    float rms[kMicArrayChannels] = {};
    float max_rms = 0.0f;
    float sum_rms = 0.0f;
    for (int ch = 0; ch < kMicArrayChannels; ++ch) {
        // RMS = sqrt(mean(sample^2))，可以理解成这一帧的平均响度。
        // 相比单点峰值，RMS 不容易被偶发尖峰带偏，更适合判断“这一路麦有没有稳定声音”。
        rms[ch] = std::sqrt(energy[ch] / frames);
        max_rms = std::max(max_rms, rms[ch]);
        sum_rms += rms[ch];
    }

    if (max_rms < kMinRmsForVoice || sum_rms <= 1.0f) {
        for (size_t i = 0; i < frames; ++i) {
            output_mic[i] = mic_data[kMicArrayChannels * i + 0];
        }
        last_confidence_ *= 0.75f;
        if (last_confidence_ < kMinConfidence) {
            has_estimate_ = false;
        }
        if (ShouldLogDebug(timestamp_ms)) {
            KORVO1_MIC_ARRAY_LOGI("inactive: fixed-main low energy frames=%u rms=[%.1f %.1f %.1f] threshold=%.1f",
                                  static_cast<unsigned>(frames), rms[0], rms[1], rms[2], kMinRmsForVoice);
        }
        return result;
    }

    const float side_max_rms = std::max(rms[1], rms[2]);
    const float main_ratio = rms[0] / std::max(side_max_rms, 1.0f);
    float confidence = Clamp((rms[0] - kMinRmsForVoice) / (kFullScaleRms - kMinRmsForVoice), 0.0f, 1.0f);
    confidence *= Clamp(main_ratio / kMainDominanceRatio, 0.0f, 1.0f);

    has_estimate_ = confidence >= kMinConfidence && main_ratio >= kMainDominanceRatio;
    last_confidence_ = confidence;

    float delays[kMicArrayChannels] = {};
    float max_arrival = -1.0e9f;
    for (int ch = 0; ch < kMicArrayChannels; ++ch) {
        const float radius = kMicDistanceMeters / std::sqrt(3.0f);
        const float arrival = -radius * std::cos(DegToRad(kMicAnglesDeg[ch])) / kSpeedOfSoundMetersPerSec;
        delays[ch] = arrival * kSampleRate;
        max_arrival = std::max(max_arrival, delays[ch]);
    }
    for (int ch = 0; ch < kMicArrayChannels; ++ch) {
        delays[ch] = max_arrival - delays[ch];
    }

    const float weights[kMicArrayChannels] = {kMainMicWeight, kSideMicWeight, kSideMicWeight};
    const float side_suppression = has_estimate_ ? 1.0f : Clamp(main_ratio, 0.15f, 0.65f);
    for (size_t i = 0; i < frames; ++i) {
        const float main = SampleWithDelay(mic_data, frames, 0, i, delays[0]);
        const float side1 = SampleWithDelay(mic_data, frames, 1, i, delays[1]);
        const float side2 = SampleWithDelay(mic_data, frames, 2, i, delays[2]);
        const float mixed = main * weights[0] + (side1 * weights[1] + side2 * weights[2]) * side_suppression;
        const float norm = weights[0] + (weights[1] + weights[2]) * side_suppression;
        output_mic[i] = ClampToInt16(mixed / norm);
    }

    result.active = has_estimate_;
    result.angle_deg = 0.0f;
    result.confidence = confidence;

    if (ShouldLogDebug(timestamp_ms)) {
        KORVO1_MIC_ARRAY_LOGI(
            "fixed-main: frames=%u rms=[%.1f %.1f %.1f] ratio=%.2f conf=%.2f suppress=%.2f delays=[%.2f %.2f %.2f] active=%d",
            static_cast<unsigned>(frames), rms[0], rms[1], rms[2], main_ratio, result.confidence, side_suppression,
            delays[0], delays[1], delays[2], result.active);
    }

    return result;
}

void Korvo1MicArrayProcessor::Reset() {
    has_estimate_ = false;
    last_confidence_ = 0.0f;
}

bool Korvo1MicArrayProcessor::ShouldLogDebug(uint32_t timestamp_ms) {
    if (timestamp_ms == 0 || last_debug_log_ms_ == 0 ||
        timestamp_ms - last_debug_log_ms_ >= kDebugLogIntervalMs) {
        last_debug_log_ms_ = timestamp_ms;
        return true;
    }
    return false;
}
