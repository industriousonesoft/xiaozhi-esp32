#ifndef AUDIO_SERVICE_ROUTE_H
#define AUDIO_SERVICE_ROUTE_H

enum class AudioProcessorOutputRoute {
    kEncodeToSendQueue,
    kPlaybackQueue,
};

inline AudioProcessorOutputRoute SelectAudioProcessorOutputRoute(bool runtime_local_playback,
                                                                 bool compile_time_local_playback) {
    return (runtime_local_playback || compile_time_local_playback)
        ? AudioProcessorOutputRoute::kPlaybackQueue
        : AudioProcessorOutputRoute::kEncodeToSendQueue;
}

#endif // AUDIO_SERVICE_ROUTE_H
