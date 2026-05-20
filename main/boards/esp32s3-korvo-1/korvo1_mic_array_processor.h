#ifndef _KORVO1_MIC_ARRAY_PROCESSOR_H_
#define _KORVO1_MIC_ARRAY_PROCESSOR_H_

#include <cstddef>
#include <cstdint>
#include <vector>

struct Korvo1MicArrayResult {
    bool active = false;
    float angle_deg = 0.0f;
    float confidence = 0.0f;
    uint32_t timestamp_ms = 0;
};

class Korvo1MicArrayProcessor {
public:
    Korvo1MicArrayProcessor() = default;
    ~Korvo1MicArrayProcessor();

    Korvo1MicArrayResult ProcessRaw(const int16_t* raw_data, size_t raw_samples,
                                    std::vector<int16_t>& output_mr, uint32_t timestamp_ms);
    Korvo1MicArrayResult Process(const int16_t* data, size_t samples, uint32_t timestamp_ms);
    void Reset();

private:
    bool has_estimate_ = false;
    float last_confidence_ = 0.0f;
    uint32_t last_debug_log_ms_ = 0;

    Korvo1MicArrayResult ProcessFixedMainMicBeamformer(const int16_t* mic_data, size_t frames,
                                                       std::vector<int16_t>& output_mic, uint32_t timestamp_ms);
    bool ShouldLogDebug(uint32_t timestamp_ms);
};

#endif // _KORVO1_MIC_ARRAY_PROCESSOR_H_
