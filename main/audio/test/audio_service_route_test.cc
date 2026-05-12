#include "../audio_service_route.h"

#include <cassert>
#include <cstdint>
#include <vector>

int main() {
    assert(SelectAudioProcessorOutputRoute(false, false) == AudioProcessorOutputRoute::kEncodeToSendQueue);
    assert(SelectAudioProcessorOutputRoute(true, false) == AudioProcessorOutputRoute::kPlaybackQueue);
    assert(SelectAudioProcessorOutputRoute(false, true) == AudioProcessorOutputRoute::kPlaybackQueue);
    assert(SelectAudioProcessorOutputRoute(true, true) == AudioProcessorOutputRoute::kPlaybackQueue);

    std::vector<int16_t> pcm = {10, -20, 20000, -20000, 0};
    auto result = ApplyPcmGainForLocalPlayback(pcm, 2);
    assert((pcm == std::vector<int16_t>{20, -40, 32767, -32768, 0}));
    assert(result.peak_before == 20000);
    assert(result.peak_after == 32768);
    assert(result.clipped_samples == 2);

    std::vector<int16_t> unity = {1, -2, 3};
    result = ApplyPcmGainForLocalPlayback(unity, 1);
    assert((unity == std::vector<int16_t>{1, -2, 3}));
    assert(result.peak_before == 3);
    assert(result.peak_after == 3);
    assert(result.clipped_samples == 0);
    return 0;
}
