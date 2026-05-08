#include "../korvo1_mic_array_processor.h"

#include <cassert>
#include <cmath>
#include <cstdint>
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

int main() {
    auto dual_mic_raw = MakeRawFrame(2, 100, 1100, 2200, 3300);
    std::vector<int16_t> dual_mic_output;
    bool dual_mic_ok = BuildKorvo1DualMicBssInput(dual_mic_raw.data(), dual_mic_raw.size(), dual_mic_output);
    assert(dual_mic_ok);
    assert(dual_mic_output.size() == 2 * 3);
    assert(dual_mic_output[0] == 1100);
    assert(dual_mic_output[1] == 2200);
    assert(dual_mic_output[2] == 100);
    assert(dual_mic_output[3] == 1100);
    assert(dual_mic_output[4] == 2200);
    assert(dual_mic_output[5] == 100);

    auto invalid_raw = MakeRawFrame(1, 100, 1100, 2200, 3300);
    invalid_raw.pop_back();
    assert(!BuildKorvo1DualMicBssInput(invalid_raw.data(), invalid_raw.size(), dual_mic_output));
    assert(dual_mic_output.empty());

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

    auto side = MakeRawFrame(160, -123, 700, 1800, 3600);
    Korvo1MicArrayProcessor side_processor;
    auto side_result = side_processor.ProcessRaw(side.data(), side.size(), output, 20);
    assert(side_result.active);
    assert(side_result.confidence > 0.2f);
    assert(RmsOfChannel(output, 2, 0) > 1800.0f);
    assert(output[1] == -123);

    return 0;
}
