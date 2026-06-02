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

// ES7210 原始 I2S 读出按 4 路 int16 lane 处理。根据 esp-skainet Korvo-1 BSP，lane0 是回声参考。
// Korvo-1 的三颗麦在 65mm 等边三角形顶点；这里选用后两颗对称麦形成二麦基线，
// 固定做正前方 broadside 定向拾音，第三颗麦不参与输出。
// 选择 raw[2]/raw[3] 的原因：这两颗麦在圆形 PCBA 边缘上构成一条相对稳定的左右基线，
// 更适合做“正前方 ±30°”的时间差门控；raw[1] 暂时不参与，避免三角阵列的几何校准误差影响固定二麦方案。
constexpr int kRawChannels = 4;
constexpr int kRawRefIndex = 0;
constexpr int kRawBeamMic0Index = 2;
constexpr int kRawBeamMic1Index = 3;

// 麦阵输出给后级 AFE 的格式固定为 M/R：一路增强后的主声源 + 一路播放参考。
constexpr int kAfeChannels = 2;
constexpr int kMicArrayChannels = 2;

// 物理参数用于把“采样点延时”换算成近似入射角：
//   max_tdoa_seconds = mic_distance / sound_speed
//   max_tdoa_samples = max_tdoa_seconds * sample_rate
// 65mm 在 48kHz 下最大约 9.1 samples；±30°对应约一半最大时间差。
constexpr float kMicDistanceMeters = 0.065f;
constexpr float kSpeedOfSoundMetersPerSec = 343.0f;
constexpr float kPickupHalfAngleDeg = 30.0f;

// Beamformer 的经验门限。这里判断的是二麦基线里最强一路的 10ms/16ms RMS。
// 太低会被底噪触发，太高会漏掉远距离/小音量说话。
constexpr float kMinRmsForVoice = 280.0f;
constexpr float kFullScaleRms = 3500.0f;
constexpr float kMinConfidence = 0.18f;
constexpr uint32_t kDebugLogIntervalMs = 500;
// 两颗麦的响度比值下限。正前方远场声源到两颗对称麦的能量应接近；
// 若差异太大，通常是近场、偏轴、遮挡、结构漏声或单通道异常，不能只看 TDOA 就判为前方声源。
constexpr float kMinPairBalance = 0.75f;

float Clamp(float value, float min_value, float max_value) {
    return std::max(min_value, std::min(value, max_value));
}

int16_t ClampToInt16(float value) {
    value = Clamp(value, -32768.0f, 32767.0f);
    return static_cast<int16_t>(std::lround(value));
}

float RadiansToDegrees(float value) {
    return value * 180.0f / 3.14159265358979323846f;
}

float SampleMicWithLag(const int16_t* mic_data, size_t frames, int ch, int index) {
    // 对齐两路麦时，边界处没有足够的历史/未来样本。这里用端点夹紧而不是丢弃整帧，
    // 可以保持输出长度严格等于输入帧数，避免破坏后级 AFE 对 M/R 通道帧长的假设。
    if (index < 0) {
        index = 0;
    } else if (index >= static_cast<int>(frames)) {
        index = static_cast<int>(frames) - 1;
    }
    return static_cast<float>(mic_data[kMicArrayChannels * index + ch]);
}
} // namespace

Korvo1MicArrayProcessor::Korvo1MicArrayProcessor(int sample_rate)
    : sample_rate_(sample_rate > 0 ? sample_rate : 16000) {
    // 采样率只影响 TDOA 搜索范围和角度换算，不改变输出给后级的通道格式。
    // 后级 AudioService 会按 codec->input_sample_rate() 把 M/R 数据重采样到 16kHz。
}

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
        // 保留 reference 的原始时序，供后级 AFE 做设备端 AEC。
        // 二麦处理只改 M 通道，不改 R 通道，避免破坏回声参考和播放信号之间的时间关系。
        ref_data[i] = raw_data[kRawChannels * i + kRawRefIndex];
        mic_data[kMicArrayChannels * i + 0] = raw_data[kRawChannels * i + kRawBeamMic0Index];
        mic_data[kMicArrayChannels * i + 1] = raw_data[kRawChannels * i + kRawBeamMic1Index];
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
    // 这里不做动态 DOA 选择，也不依赖乐鑫 MASE。当前实现只使用两颗对称麦，
    // 固定为正前方 broadside：两路同相时增强，左右侧不均衡时衰减。
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
        // 低能量帧通常是静音或底噪。此时不做角度估计，直接输出两麦平均值；
        // 这样能保持背景噪声平滑，也避免底噪随机相关导致 active 状态抖动。
        for (size_t i = 0; i < frames; ++i) {
            const float mic0 = static_cast<float>(mic_data[kMicArrayChannels * i + 0]);
            const float mic1 = static_cast<float>(mic_data[kMicArrayChannels * i + 1]);
            output_mic[i] = ClampToInt16((mic0 + mic1) * 0.5f);
        }
        last_confidence_ *= 0.75f;
        if (last_confidence_ < kMinConfidence) {
            has_estimate_ = false;
        }
        if (ShouldLogDebug(timestamp_ms)) {
            KORVO1_MIC_ARRAY_LOGI("inactive: two-mic low energy frames=%u rms=[%.1f %.1f] threshold=%.1f",
                                  static_cast<unsigned>(frames), rms[0], rms[1], kMinRmsForVoice);
        }
        return result;
    }

    const int max_tdoa_samples = std::max(1, static_cast<int>(
        std::lround(kMicDistanceMeters / kSpeedOfSoundMetersPerSec * sample_rate_)));
    const int pickup_tdoa_samples = std::max(1, static_cast<int>(
        std::lround(max_tdoa_samples * std::sin(kPickupHalfAngleDeg * 3.14159265358979323846f / 180.0f))));

    // 用归一化互相关在 [-max_tdoa_samples, +max_tdoa_samples] 内搜索两麦最可能的时间差。
    // lag 的定义：mic1 取 i + lag 与 mic0 的 i 对齐；best_lag 越大，表示两路需要越大的样本偏移才能相干。
    // 这里使用整数采样点搜索，48kHz 下 65mm 基线已有约 9 个最大采样点，足够做 ±30°门控。
    int best_lag = 0;
    float best_corr = 0.0f;
    for (int lag = -max_tdoa_samples; lag <= max_tdoa_samples; ++lag) {
        double sum0 = 0.0;
        double sum1 = 0.0;
        double sum00 = 0.0;
        double sum11 = 0.0;
        double sum01 = 0.0;
        int count = 0;

        const int start = std::max(0, -lag);
        const int end = std::min(static_cast<int>(frames), static_cast<int>(frames) - lag);
        for (int i = start; i < end; ++i) {
            // 只在两路都有有效样本的重叠区间计算互相关，避免边界补样影响 TDOA 判断。
            const double mic0 = static_cast<double>(mic_data[kMicArrayChannels * i + 0]);
            const double mic1 = static_cast<double>(mic_data[kMicArrayChannels * (i + lag) + 1]);
            sum0 += mic0;
            sum1 += mic1;
            sum00 += mic0 * mic0;
            sum11 += mic1 * mic1;
            sum01 += mic0 * mic1;
            ++count;
        }

        if (count <= 1) {
            continue;
        }

        const double mean0 = sum0 / count;
        const double mean1 = sum1 / count;
        const double var0 = sum00 - count * mean0 * mean0;
        const double var1 = sum11 - count * mean1 * mean1;
        float corr = 1.0f;
        if (var0 > 1.0 && var1 > 1.0) {
            // Pearson 相关系数会先去掉直流偏置，再按能量归一化。
            // 这样 AGC、通道增益差和轻微 DC 偏移不容易把 TDOA 搜索带偏。
            const double cov = sum01 - count * mean0 * mean1;
            corr = static_cast<float>(cov / std::sqrt(var0 * var1));
            corr = Clamp(corr, -1.0f, 1.0f);
        } else if (lag != 0) {
            corr = 0.0f;
        }

        if (corr > best_corr) {
            best_corr = corr;
            best_lag = lag;
        }
    }

    // 把整数 TDOA 转成近似入射角。这里使用线性二麦远场模型：
    //   sin(theta) = tdoa / max_tdoa
    // theta=0 表示正前方 broadside；|theta| 越大，越偏离正前方。
    const float tdoa_ratio = Clamp(static_cast<float>(best_lag) / max_tdoa_samples, -1.0f, 1.0f);
    const float angle_deg = RadiansToDegrees(std::asin(tdoa_ratio));
    const float angle_abs = std::abs(angle_deg);

    // pair_balance 是两麦响度平衡度，angle_score 是 60°拾音角的软门控分数。
    // 角度在 ±30°内时 angle_score=1；超过 ±30°后逐步降到 0，而不是突然静音，
    // 这样真实设备上不会因为单帧估计波动产生明显断续。
    const float min_rms = std::min(rms[0], rms[1]);
    const float pair_balance = min_rms / std::max(max_rms, 1.0f);
    const float balance_score = Clamp(pair_balance / kMinPairBalance, 0.0f, 1.0f);
    const float angle_score = angle_abs <= kPickupHalfAngleDeg
        ? 1.0f
        : Clamp(1.0f - (angle_abs - kPickupHalfAngleDeg) / (90.0f - kPickupHalfAngleDeg), 0.0f, 1.0f);
    float confidence = Clamp((max_rms - kMinRmsForVoice) / (kFullScaleRms - kMinRmsForVoice), 0.0f, 1.0f);
    // 置信度同时依赖响度、两麦能量平衡、互相关强度和角度门控。
    // balance_score 使用四次方是故意的：能量严重不平衡时，要强力压低近场/侧向误判。
    confidence *= balance_score * balance_score * balance_score * balance_score * best_corr * angle_score * angle_score;

    // active 代表“这一帧可以认为是 60°拾音角内的有效前方声源”。
    // 除了角度落在门限内，还要求两麦能量足够平衡、互相关足够强，避免噪声或单通道大音量触发方向显示。
    has_estimate_ = confidence >= kMinConfidence && pair_balance >= kMinPairBalance &&
        best_corr >= 0.35f && std::abs(best_lag) <= pickup_tdoa_samples;
    last_confidence_ = confidence;

    // 对 active 帧，按 best_lag 对齐后直接平均，得到前方增强的 M 通道。
    // 对非 active 帧，不完全静音，而是按角度和能量平衡做 0.15~0.65 的衰减；
    // 这样仍保留少量环境声，减少 VAD/AEC 输入突变，同时实现角外人声压低。
    const float off_axis_suppression = has_estimate_ ? 1.0f : Clamp(pair_balance * angle_score, 0.15f, 0.65f);
    for (size_t i = 0; i < frames; ++i) {
        const float mic0 = SampleMicWithLag(mic_data, frames, 0, static_cast<int>(i));
        const float mic1 = SampleMicWithLag(mic_data, frames, 1, static_cast<int>(i) + best_lag);
        output_mic[i] = ClampToInt16((mic0 + mic1) * 0.5f * off_axis_suppression);
    }

    result.active = has_estimate_;
    result.angle_deg = angle_deg;
    result.confidence = confidence;

    if (ShouldLogDebug(timestamp_ms)) {
        KORVO1_MIC_ARRAY_LOGI(
            "two-mic-front: frames=%u baseline=%.3fm sample_rate=%d lag=%d/%d gate=%d angle=%.1f rms=[%.1f %.1f] balance=%.2f corr=%.2f conf=%.2f suppress=%.2f active=%d",
            static_cast<unsigned>(frames), kMicDistanceMeters, sample_rate_, best_lag, max_tdoa_samples,
            pickup_tdoa_samples, angle_deg, rms[0], rms[1], pair_balance, best_corr, result.confidence,
            off_axis_suppression, result.active);
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
