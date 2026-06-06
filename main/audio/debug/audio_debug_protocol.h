#ifndef AUDIO_DEBUG_PROTOCOL_H
#define AUDIO_DEBUG_PROTOCOL_H

#include <cstddef>
#include <cstdint>
#include <vector>

constexpr uint32_t kAudioDebugMagic = 0x47424441; // "ADBG" in little endian.
constexpr uint8_t kAudioDebugProtocolVersion = 1;
constexpr size_t kAudioDebugHeaderSize = 40;
constexpr size_t kAudioDebugMaxDatagramSize = 1472;
constexpr uint8_t kAudioDebugSampleFormatS16Le = 1;

enum class AudioDebugStream : uint8_t {
    kRawMics = 1,
    kBeam = 2,
    kAfe = 3,
    kDirection = 4,
};

struct AudioDebugPacketHeader {
    AudioDebugStream stream = AudioDebugStream::kRawMics;
    uint32_t session_id = 0;
    uint32_t sequence = 0;
    uint64_t timestamp_us = 0;
    uint32_t sample_rate = 0;
    uint16_t frame_count = 0;
    uint8_t channels = 0;
    uint8_t sample_format = kAudioDebugSampleFormatS16Le;
    uint16_t payload_size = 0;
    uint16_t flags = 0;
};

struct AudioDebugPcmChunk {
    size_t frame_offset = 0;
    size_t frame_count = 0;
};

bool EncodeAudioDebugHeader(const AudioDebugPacketHeader& header, uint8_t* output, size_t output_size);
bool DecodeAudioDebugHeader(const uint8_t* data, size_t size, AudioDebugPacketHeader& header);
size_t AudioDebugMaxFramesPerPacket(uint8_t channels);
std::vector<AudioDebugPcmChunk> SplitAudioDebugPcm(const int16_t* samples, size_t frames, uint8_t channels);

class AudioDebugPacketBuilder {
public:
    explicit AudioDebugPacketBuilder(uint32_t session_id);

    std::vector<std::vector<uint8_t>> BuildPcmPackets(
        AudioDebugStream stream, const int16_t* samples, size_t frames, uint8_t channels,
        uint32_t sample_rate, uint64_t timestamp_us);
    std::vector<uint8_t> BuildDirectionPacket(
        float angle_deg, float confidence, bool active, uint64_t timestamp_us);

private:
    uint32_t session_id_;
    uint32_t sequences_[4] = {};

    uint32_t NextSequence(AudioDebugStream stream);
};

#endif
