#include "../audio_service_route.h"

#include <cassert>

int main() {
    assert(SelectAudioProcessorOutputRoute(false, false) == AudioProcessorOutputRoute::kEncodeToSendQueue);
    assert(SelectAudioProcessorOutputRoute(true, false) == AudioProcessorOutputRoute::kPlaybackQueue);
    assert(SelectAudioProcessorOutputRoute(false, true) == AudioProcessorOutputRoute::kPlaybackQueue);
    assert(SelectAudioProcessorOutputRoute(true, true) == AudioProcessorOutputRoute::kPlaybackQueue);
    return 0;
}
