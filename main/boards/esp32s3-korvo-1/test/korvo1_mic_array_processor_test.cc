#include "../korvo1_mic_array_processor.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <vector>

static std::vector<int16_t> MakeRawFrame(int frames, int16_t ref, int16_t mic0, int16_t mic1, int16_t mic2) {
    std::vector<int16_t> data(frames * 4);
    for (int i = 0; i < frames; ++i) {
        data[4 * i + 0] = ref;
        data[4 * i + 1] = mic0;
        data[4 * i + 2] = mic1;
        data[4 * i + 3] = mic2;
    }
    return data;
}

static float RmsOfChannel(const std::vector<int16_t>& data, int channels, int index) {
    double energy = 0.0;
    int frames = data.size() / channels;
    for (int i = 0; i < frames; ++i) {
        float sample = static_cast<float>(data[channels * i + index]);
        energy += sample * sample;
    }
    return std::sqrt(energy / frames);
}

static std::vector<int16_t> MakeRawSine(int frames, int16_t ref, float mic0_gain, float mic1_gain, float mic2_gain) {
    std::vector<int16_t> data(frames * 4);
    constexpr float kPi = 3.14159265358979323846f;
    for (int i = 0; i < frames; ++i) {
        float wave = std::sin(2.0f * kPi * i / 32.0f);
        data[4 * i + 0] = ref;
        data[4 * i + 1] = static_cast<int16_t>(std::lround(wave * mic0_gain));
        data[4 * i + 2] = static_cast<int16_t>(std::lround(wave * mic1_gain));
        data[4 * i + 3] = static_cast<int16_t>(std::lround(wave * mic2_gain));
    }
    return data;
}

static std::vector<int16_t> MakeRawShiftedSignal(int frames, int16_t ref, int lag_samples) {
    std::vector<int16_t> source(frames + std::abs(lag_samples) + 1);
    for (size_t i = 0; i < source.size(); ++i) {
        float wave = 2100.0f * std::sin(2.0f * 3.14159265358979323846f * i / 23.0f);
        wave += 900.0f * std::sin(2.0f * 3.14159265358979323846f * i / 11.0f);
        source[i] = static_cast<int16_t>(std::lround(wave));
    }

    std::vector<int16_t> data(frames * 4);
    for (int i = 0; i < frames; ++i) {
        int mic0_index = i + std::max(lag_samples, 0);
        int mic1_index = i + std::max(-lag_samples, 0);
        data[4 * i + 0] = ref;
        data[4 * i + 1] = 0;
        data[4 * i + 2] = source[mic0_index];
        data[4 * i + 3] = source[mic1_index];
    }
    return data;
}

int main() {
    Korvo1MicArrayProcessor processor;

    auto silence = MakeRawFrame(160, 100, 20, 18, 22);
    std::vector<int16_t> output;
    auto silent_result = processor.ProcessRaw(silence.data(), silence.size(), output, 0);
    assert(output.size() == 160 * 2);
    assert(!silent_result.active);
    assert(output[0] == 20);
    assert(output[1] == 100);

    auto front = MakeRawFrame(160, 321, 2600, 2600, 2600);
    auto front_result = processor.ProcessRaw(front.data(), front.size(), output, 10);
    assert(output.size() == 160 * 2);
    assert(front_result.active);
    assert(front_result.angle_deg < 30.0f || front_result.angle_deg > 330.0f);
    assert(output[0] == 2600);
    assert(output[1] == 321);

    auto ignored_mic = MakeRawSine(160, 77, 9000.0f, 2000.0f, 2000.0f);
    Korvo1MicArrayProcessor ignored_mic_processor;
    auto ignored_mic_result = ignored_mic_processor.ProcessRaw(ignored_mic.data(), ignored_mic.size(), output, 15);
    assert(ignored_mic_result.active);
    assert(RmsOfChannel(output, 2, 0) < 1800.0f);
    assert(std::abs(output[1] - 77) == 0);

    auto side = MakeRawFrame(160, -123, 700, 1800, 3600);
    Korvo1MicArrayProcessor side_processor;
    auto side_result = side_processor.ProcessRaw(side.data(), side.size(), output, 20);
    assert(!side_result.active);
    assert(side_result.confidence < 0.2f);
    assert(RmsOfChannel(output, 2, 0) < 2600.0f);
    assert(output[1] == -123);

    auto mic1 = MakeRawFrame(160, 456, 900, 3600, 3600);
    Korvo1MicArrayProcessor mic1_processor;
    auto mic1_result = mic1_processor.ProcessRaw(mic1.data(), mic1.size(), output, 30);
    assert(mic1_result.active);
    assert(mic1_result.confidence > 0.2f);
    assert(RmsOfChannel(output, 2, 0) > 2600.0f);
    assert(output[1] == 456);

    Korvo1MicArrayProcessor processor_48k(48000);
    auto inside_60deg = MakeRawShiftedSignal(480, 111, 4);
    auto inside_result = processor_48k.ProcessRaw(inside_60deg.data(), inside_60deg.size(), output, 40);
    assert(inside_result.active);
    assert(std::abs(inside_result.angle_deg) <= 35.0f);
    assert(inside_result.confidence > 0.25f);
    assert(RmsOfChannel(output, 2, 0) > 1400.0f);
    assert(output[1] == 111);

    auto outside_60deg = MakeRawShiftedSignal(480, -222, 7);
    auto outside_result = processor_48k.ProcessRaw(outside_60deg.data(), outside_60deg.size(), output, 50);
    assert(!outside_result.active);
    assert(std::abs(outside_result.angle_deg) > 35.0f);
    assert(outside_result.confidence < 0.2f);
    assert(RmsOfChannel(output, 2, 0) < 1200.0f);
    assert(output[1] == -222);

    return 0;
}
